import matplotlib
matplotlib.use("Agg")

import board
import busio
import adafruit_bno055
import serial
import time
import threading
import math
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import numpy as np
import cv2
from path_planner import PathPlanner
from obstacle_detect import Obstacle_Detector



class RobotController:
    def __init__(self):

                         # Initialize IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

                        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.flush()

                        # Initialize camera connection
                #Camera initialization
        self.cam = cv2.VideoCapture(0)

        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


        self.yaw = None
        self.left_count = None
        self.right_count = None
        self.frame = None
        self.obs_dist = None

        self.imu_thread = threading.Thread(target=self.read_imu, daemon=True)
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.camera_thread = threading.Thread(target=self.read_camera,daemon=True)


        self.Kp_fwd,self.Ki_fwd,self.Kd_fwd = [5.0,0.0,0.0]
        self.Kp_enc,self.Ki_enc,self.Kd_enc = [1.0,0.0,0.0]
        self.Kp_yaw,self.Ki_yaw,self.Kd_yaw = [700.0,0.0,0.0]

        self.Kp_turn,self.Ki_turn,self.Kd_turn = [300.0,0.,200.]

                # Initialize GPIO (same as your motor_driver.py)
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)

        self.pwm_right = GPIO.PWM(self.ENA, 100)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_right.start(0)
        self.pwm_left.start(0)

        self.ppr = 11
        self.circumference = 18.85

        self.length = 19.3
        self.width = 11

        self.robot_rad = math.sqrt((self.length)**2 + (self.width)**2)

        self.clearance_dist = self.robot_rad


        self.threads_running = True

        self.imu_thread.start()
        self.serial_thread.start()
        self.camera_thread.start()

        self.trajectory = []
        self.x = 60.0
        self.y = 60.0

        self.trajectory.append((self.x,self.y))

        self.LC_last = 0
        self.RC_last = 0

        self.first_reading = True




        time.sleep(20000) # For the sensors to load the data into the variables
        #initialize variables




    def read_imu(self):
        while(self.threads_running):
            accel = self.imu.acceleration
            gyro = self.imu.gyro
            euler = self.imu.euler
            print(f"Accel: {accel}")
            print(f"Gyro: {gyro}")
            print(f"Euler: {euler}")
            time.sleep(1)
            # while(self.threads_running):
            #     yaw = self.imu.euler[0] #following ros convention
            #     if ( yaw is not None):
            #         self.yaw = self.normalize_angle_in_rad(-math.radians(yaw))
            #     time.sleep(0.01)


    def read_serial(self):
        try:
            while(self.threads_running):
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    values = line.split(",")

                    if len(values) >= 2:
                        self.left_count = float(values[0])
                        self.right_count = float(values[1])
                        self.obs_dist = float(values[2])
                time.sleep(0.01)

        except Exception as e:
            print("Encoder read error:", e)

    def read_camera(self):
        try:
            while(self.threads_running):
                ret, frame = self.cam.read()

                if ret:
                    self.frame = cv2.rotate(frame, cv2.ROTATE_180)

        except Exception as e:
            print("Frame capture error: ",e)

    def publish_plot(self):
        # print(self.trajectory)
        x_coords = [int(point[0]) for point in self.trajectory]
        y_coords = [int(point[1]) for point in self.trajectory]

        plt.plot(x_coords[0],y_coords[0],'go',markersize=10,label="Start")
        plt.plot(x_coords[-1],y_coords[-1],'ro',markersize=10,label="End")


        plt.plot(np.array(x_coords),np.array(y_coords),'b-',linewidth=3,label="Robot Path")


        plt.grid(True)
        plt.axis('equal')

        plt.title("Trajectory")

        plt.xlabel("X-Coords")
        plt.ylabel("Y-Coords")

        plt.legend()

        plt.savefig('robot_trajectory.png',dpi=300,bbox_inches="tight")


        # plt.show()




    def normalize_angle_in_rad(self,angle):
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi
        return angle


    def turn_by_angle(self,angle):
        start_angle = self.yaw
        angle = self.normalize_angle_in_rad(math.radians(angle))
        target_angle = self.normalize_angle_in_rad(start_angle + angle)

        err_angle = self.normalize_angle_in_rad(target_angle - start_angle)
        err_angle_sum = 0
        err_angle_diff = 0
        err_last_angle = err_angle

        while(True):
            time.sleep(0.05)
            current_angle = self.yaw

            err_angle = self.normalize_angle_in_rad(target_angle - current_angle)
            err_angle_sum += err_angle
            err_angle_diff = self.normalize_angle_in_rad(err_angle - err_last_angle)

            err_last_angle = err_angle

            turn_speed = self.Kp_turn*err_angle + self.Ki_turn*err_angle_sum + self.Kd_turn*err_angle_diff

            turn_speed = self.clip_speed(turn_speed)
            right_speed = turn_speed
            left_speed = -right_speed

            print(f"target_angle: {math.degrees(target_angle)}, Current_angle: {math.degrees(current_angle)}, Error: {(err_angle)}, left_speed: {left_speed}, right_speed: {right_speed}")



            if( -0.06936<err_angle<0.06936 ):
                print(f"Error: {err_angle} - Breaking.......")
                break

            self.set_motor_speeds(left_speed,right_speed)

        self.set_motor_speeds(0,0)
        time.sleep(1)
       
        
    def clip_speed(self,speed):
        speed = min(80,speed)
        speed = max(-80,speed)

        return speed

    def set_motor_speeds(self,left_speed,right_speed):
        left_speed = self.clip_speed(left_speed)
        right_speed = self.clip_speed(right_speed)



        if left_speed > 0:
            GPIO.output(self.IN3,GPIO.HIGH)
            GPIO.output(self.IN4,GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(left_speed)
        elif left_speed < 0:
            GPIO.output(self.IN3,GPIO.LOW)
            GPIO.output(self.IN4,GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(left_speed))
        else:
            self.pwm_left.ChangeDutyCycle(0)

        if right_speed > 0:
            GPIO.output(self.IN1,GPIO.LOW)
            GPIO.output(self.IN2,GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(right_speed)
        elif right_speed < 0:
            GPIO.output(self.IN1,GPIO.HIGH)
            GPIO.output(self.IN2,GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(abs(right_speed))
        else:
            self.pwm_right.ChangeDutyCycle(0)

    def calculate_distance_covered(self,LC,RC):
        left_distance = (self.circumference/self.ppr)*LC
        right_distance = (self.circumference/self.ppr)*RC

        return (left_distance + right_distance)/2


    def forward_controller(self,max_dist):
        start_count_left = self.left_count
        start_count_right = self.right_count

        start_yaw = self.yaw
        avg_yaw = start_yaw
        itr = 0

        err_dist = max_dist
        err_dist_sum = 0.
        err_dist_diff = 0.
        err_dist_last = err_dist

        err_enc_sum = 0.
        err_enc_diff = 0.
        err_enc_last = 0.

        err_yaw_sum = 0.
        err_yaw_diff = 0.
        err_yaw_last = 0.

        #Setting initial speed
        speed_sum = self.Kp_fwd*err_dist + self.Ki_fwd*err_dist_sum + self.Kd_fwd*err_dist_diff
        speed_sum = self.clip_speed(speed_sum)
        speed_L = speed_sum/2 # These are control speeds, not the resultant robot speeds
        speed_R = speed_sum/2
        self.set_motor_speeds(speed_L,speed_R)

        self.tol_d = 10

        while True:
            time.sleep(0.05)
            #Track imu
            itr += 1
            avg_yaw = (avg_yaw * (itr-1) + self.yaw)/ itr


            curr_yaw = self.yaw
            err_yaw = self.normalize_angle_in_rad(start_yaw - curr_yaw)
            err_yaw_sum += self.normalize_angle_in_rad(err_yaw)
            err_yaw_diff = self.normalize_angle_in_rad(err_yaw - err_yaw_last)
            err_yaw_last = err_yaw

            #Track encoder
            LC = self.left_count - start_count_left
            RC = self.right_count - start_count_right

            err_enc = LC - RC
            err_enc_sum += err_enc
            err_enc_diff = err_enc - err_enc_last
            err_enc_last = err_enc

            #Track distance
            d = self.calculate_distance_covered(LC,RC)

            err_dist = max_dist - d

            err_dist_sum += err_dist
            err_dist_diff = err_dist - err_dist_last
            err_dist_last = err_dist

            speed_sum = self.Kp_fwd*err_dist + self.Ki_fwd*err_dist_sum + self.Kd_fwd*err_dist_diff
            speed_sum = self.clip_speed(speed_sum)

            speed_diff = self.Kp_enc*err_enc + self.Ki_enc*err_enc_sum + self.Kd_enc*err_enc_diff + \
                         self.Kp_yaw*err_yaw + self.Ki_yaw*err_yaw_sum + self.Kd_yaw*err_yaw_diff

            speed_diff = self.clip_speed(speed_diff)

            speed_R = (speed_sum + speed_diff)/2

            speed_R = self.clip_speed(speed_R)

            speed_L = speed_sum - speed_R

            speed_L = self.clip_speed(speed_L)

            print(f"Distance Covered: {d:.1f}, Error:{err_dist:.1f}, Speed : L={speed_L:.1f}, R={speed_R:.1f}")

            print(f"Start Theta: {math.degrees(start_yaw)}, Current Theta: {math.degrees(curr_yaw)},  Error: {math.degrees(err_yaw)}")



            if( self.obs_dist < self.clearance_dist or err_dist<self.tol_d):
                print("Stop command sent......")
                self.set_motor_speeds(0,0)
                time.sleep(2)
                LC = self.left_count - start_count_left    # Calculating again considering the inertia
                RC = self.right_count - start_count_right
                d = self.calculate_distance_covered(LC,RC)
                self.x += d*math.cos(avg_yaw)
                self.y += d*math.sin(avg_yaw)
                self.trajectory.append((self.x,self.y))
                if(err_dist > self.obs_dist):
                  return False
                break
            self.set_motor_speeds(speed_L,speed_R)

        return True

    def follow_path(self,path):

        print(path)
        for i in range(len(path)-1):
            current = path[i]
            next = path[i+1]

            theta = math.atan2((next[1]-current[1]),(next[0]-current[0]))
            angle = theta - self.yaw

            # print(math.degrees(angle))
            print(f'Current Angle: {math.degrees(self.yaw)}, Turn Angle: {math.degrees(angle)}')
            self.turn_by_angle(math.degrees(angle))
            time.sleep(1)

            print(f'Successfully made the turn - Current yaw is {math.degrees(self.yaw)} ')



            dist = ((next[0]-current[0])**2 + (next[1]-current[1])**2)**0.5

            ret_val = self.forward_controller(dist-15)

            if(not ret_val):
              print(f"Found an obstacle, aborting mission......")
              return False


            print(f'Successfully made the forward movement.....')
            time.sleep(1)

        return True


    def robot_to_world_transform(self,x_r,y_r,robot_x,robot_y,robot_theta):

        # Apply rotation + translation
        x_w = x_r * math.cos(robot_theta) - y_r * math.sin(robot_theta) + robot_x
        y_w = x_r * math.sin(robot_theta) + y_r * math.cos(robot_theta) + robot_y

        print(f"World coords of the obstacle is: ({x_w},{y_w})")

        return [x_w, y_w]



    def cleanup(self):
        self.threads_running = False
        GPIO.cleanup()
        self.ser.close()
        self.pwm_right.stop()
        self.pwm_left.stop()


def main(args=None):
    try:
      robot_controller = RobotController()
      end_node = (300,240)
      obstacles = []
      while(True):
          print("Running.....")

    #   robot_controller.forward_controller(60)

    #   while True:
    #     obstacle_detector = Obstacle_Detector()
    #     obstacle_detector.get_all_obstacles(robot_controller.frame)

    #     robot_x,robot_y,robot_theta = robot_controller.x,robot_controller.y,robot_controller.yaw

        
    #     all_obstacles = obstacle_detector.obstacles

    #     print(f"{len(all_obstacles)} black obstacles detected by camera.......")

    #     for obstacle in all_obstacles:
    #       print(obstacle)
    #       print(obstacle[0],obstacle[1])
    #       d = ((obstacle[0])**2 + (obstacle[1])**2)**(1/2)
    #       dist_to_target = ((robot_controller.x - end_node[0])**2 + (robot_controller.y - end_node[1])**2)**0.5

    #       print(f"Distance of obstacle from robot - {d}....... Distance to the final target - {dist_to_target}")

    #       if (d<dist_to_target):
    #         obs_world = robot_controller.robot_to_world_transform(obstacle[0],obstacle[1],robot_x,robot_y,robot_theta)
    #         obs_to_rect = {
    #           "type":"rectangle",
    #           "name":"unknown",
    #           "x_start":obs_world[0],
    #           "x_end":obs_world[0]+robot_controller.clearance_dist,
    #           "y_start":obs_world[1]-robot_controller.clearance_dist,
    #           "y_end":obs_world[1]+robot_controller.clearance_dist
    #           }
    #         obstacles.append(obs_to_rect)


    #     room_x_start = robot_controller.x
    #     room_x_end = 350

    #     room_y_start = robot_controller.y
    #     room_y_end = 300


    #     path_planner = PathPlanner(room_x_start,room_x_end,room_y_start,room_y_end,obstacles)
    #     start_node = (robot_controller.x,robot_controller.y)
    #     nodes = path_planner.generate_prm_nodes(50,start_node,end_node)
    #     connections = path_planner.build_roadmap(nodes,5)
    #     # print(connections)

    #     path = path_planner.a_star_search(start_node,connections,end_node,nodes)

    #     trimmed = path_planner.trim_path(path)

    #     print("Trimmed path is being displayed here below.........")
    #     print(trimmed)

    #     # path_planner.generate_plot(nodes,connections,path,trimmed,robot_controller.trajectory)

    #     success = robot_controller.follow_path(trimmed)

    #     print(robot_controller.x,robot_controller.y)

    #     path_planner.generate_plot(nodes,connections,path,trimmed,robot_controller.trajectory)
    #     print("Plot has been made and saved.......")


    #     if(success):
    #       break




    except KeyboardInterrupt:
        print("Stopping.....")
    finally:
        robot_controller.cleanup()
        # time.sleep(2)
        # robot_controller.publish_plot()

if __name__ == "__main__":
    main()