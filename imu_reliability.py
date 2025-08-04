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

        self.last_time = time.time()
        self.start_time = time.time()

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

        self.trajectory = []
        self.x = 60.0
        self.y = 60.0

        self.trajectory.append((self.x,self.y))

        self.LC_last = 0
        self.RC_last = 0

        self.first_reading = True

        #imu state variable
        self.imu_x = 60.0
        self.imu_y = 60.0
        self.vel_x = 0.0
        self.vel_y = 0.0

        self.imu_path = []
        self.raw_accel_data = []
        self.time_stamps = []

        

        self.noise_analysis = None

        self.threads_running = True

        self.imu_thread = threading.Thread(target=self.read_imu, daemon=True)
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.camera_thread = threading.Thread(target=self.read_camera,daemon=True)

      

        self.imu_thread.start()
        self.serial_thread.start()
        self.camera_thread.start()




        time.sleep(2) # For the sensors to load the data into the variables
        #initialize variables




    def read_imu(self):
        try:
            while(self.threads_running):
                current_time = time.time()
                dt = current_time - self.last_time

                
                accel = self.imu.linear_acceleration
                gyro = self.imu.gyro
                euler = self.imu.euler

                if euler[0] is not None:
                    self.yaw = self.normalize_angle_in_rad(-math.radians(euler[0]))


                if accel is None or euler[0] is None:
                    continue
                
                

        
                ax_robot = accel[0] if accel[0] is not None else 0.0
                ay_robot = accel[1] if accel[1] is not None else 0.0

                print(ax_robot,ay_robot)

                #Rotating to world frame
                ax_world = ax_robot * math.cos(self.yaw) - ay_robot * math.sin(self.yaw)
                ay_world = ax_robot * math.sin(self.yaw) + ay_robot * math.cos(self.yaw)

                self.vel_x += ax_world * dt
                self.vel_y += ay_world*dt

                self.imu_x += self.vel_x*dt
                self.imu_y += self.vel_y*dt

                self.imu_path.append((self.imu_x,self.imu_y))
                self.raw_accel_data.append((ax_world,ay_world,accel[2] if accel[2] is not None else 0.0))
                self.time_stamps.append((current_time - self.start_time))

                self.last_time = current_time

                time.sleep(0.01)

        except Exception as e:
            print(f"Error:{e}")

    def analyze_noise_and_reliability(self):

        accel_data = np.array(self.raw_accel_data)
        imu_path_array = np.array(self.imu_path)
        encoder_path_array = np.array(self.trajectory)


        ax_std = np.std(accel_data[:,0])
        ay_std = np.std(accel_data[:,1])
        ax_mean = np.mean(accel_data[:,0])
        ay_mean = np.mean(accel_data[:,1])

       # Store analysis results
        self.noise_analysis = {
            'accel_x_std': ax_std,
            'accel_y_std': ay_std,
            'accel_x_mean': ax_mean,
            'accel_y_mean': ay_mean,
            'signal_to_noise_x': abs(ax_mean) / ax_std if ax_std > 0 else 0,
            'signal_to_noise_y': abs(ay_mean) / ay_std if ay_std > 0 else 0,
            'total_samples': len(accel_data)
        }

                # Print analysis
        print('\n' + '='*50)
        print('📊 NOISE AND RELIABILITY ANALYSIS')
        print('='*50)
        print(f'🔢 Total samples: {self.noise_analysis["total_samples"]}')
        print(f'📈 Accel X: mean={ax_mean:.4f}, std={ax_std:.4f} m/s²')
        print(f'📈 Accel Y: mean={ay_mean:.4f}, std={ay_std:.4f} m/s²')
        # print(f'📏 Final position error: {position_error:.1f}cm')
        print(f'📡 Signal-to-noise ratio X: {self.noise_analysis["signal_to_noise_x"]:.2f}')
        print(f'📡 Signal-to-noise ratio Y: {self.noise_analysis["signal_to_noise_y"]:.2f}')

         # Provide recommendations
        self.provide_integration_recommendation()

           

    def provide_integration_recommendation(self):
        snr_x = self.noise_analysis['signal_to_noise_x']
        snr_y = self.noise_analysis['signal_to_noise_y']

        if snr_x <2 or snr_y <2:
            print("❌ DON\'T INTEGRATE: Signal-to-noise ratio too low")
        else:
            print("INTEGRATE: IMU shows good reliability!")


    def noise_plot(self):
        try:
            fig,((ax1,ax2)) = plt.subplots(1,2,figsize=(15,12))

            imu_x = [p[0] for p in self.imu_path]
            imu_y = [p[1] for p in self.imu_path]
            ax1.plot(imu_x,imu_y,'r-',label="IMU PATH", linewidth=2)

            enc_x = [p[0] for p in self.trajectory]
            enc_y = [p[1] for p in self.trajectory]
            ax1.plot(enc_x,enc_y,'b-',label="Encoder Path",linewidth=2)

            ax1.set_xlabel("X (cm)")
            ax1.set_ylabel("Y (cm)")
            ax1.set_title("Path Comparison: IMU vs Encoder")
            ax1.legend()
            ax1.grid(True)
            ax1.axis('equal')

            #Plot2 - accleration data
            accel_arr = np.array(self.raw_accel_data)
            ax2.plot(self.time_stamps,accel_arr[:,0],'r-',label="Accel X")
            ax2.plot(self.time_stamps,accel_arr[:,1],'g-',label="Accel Y")
            ax2.set_xlabel("Time(s)")
            ax2.set_ylabel("Acceleration")
            ax2.legend()
            ax2.grid(True)

            plt.tight_layout()
            plt.savefig("imu_reliability.png",dpi=300,bbox_inches="tight")
            plt.close()

            print("Plot is marked and saved....")

        except Exception as e:
            print(e)

    # def read_imu(self):
    #     """DEBUGGED IMU integration with detailed logging"""
    #     print("🚀 Starting IMU thread...")
        
    #     # Initialize local variables
    #     sample_count = 0
    #     dt_issues = 0
    #     accel_readings = 0
        
    #     try:
    #         while(self.threads_running):
    #             current_time = time.time()
    #             dt = current_time - self.last_time
    #             sample_count += 1
                
    #             # BUG FIX #1: Validate dt
    #             if dt > 0.1:  # More than 100ms gap
    #                 print(f"⚠️ Large dt detected: {dt*1000:.1f}ms (sample {sample_count})")
    #                 dt_issues += 1
    #                 if dt > 1.0:  # More than 1 second - skip this reading
    #                     self.last_time = current_time
    #                     continue
                
    #             if dt < 0.001:  # Less than 1ms - too fast
    #                 time.sleep(0.001)
    #                 continue
                    
    #             # Read sensors
    #             accel = self.imu.linear_acceleration
    #             gyro = self.imu.gyro
    #             euler = self.imu.euler

    #             # BUG FIX #2: Validate ALL sensor data before using
    #             if euler is None or euler[0] is None:
    #                 if sample_count % 50 == 0:  # Log occasionally
    #                     print(f"⚠️ Invalid euler data at sample {sample_count}")
    #                 time.sleep(0.01)
    #                 continue
                    
    #             if accel is None or any(x is None for x in accel):
    #                 if sample_count % 50 == 0:
    #                     print(f"⚠️ Invalid accel data at sample {sample_count}")
    #                 time.sleep(0.01)
    #                 continue

    #             # Update yaw (now safe)
    #             new_yaw = self.normalize_angle_in_rad(-math.radians(euler[0]))
                
    #             # BUG FIX #3: Initialize yaw on first valid reading
    #             if self.yaw is None:
    #                 self.yaw = new_yaw
    #                 print(f"✅ Yaw initialized: {math.degrees(new_yaw):.1f}°")
    #                 self.last_time = current_time
    #                 continue
    #             else:
    #                 self.yaw = new_yaw

    #             # Get accelerations
    #             ax_robot = accel[0]
    #             ay_robot = accel[1] 
    #             az_robot = accel[2]
    #             accel_readings += 1
                
    #             # BUG FIX #4: Apply noise threshold
    #             NOISE_THRESHOLD = 0.02  # Ignore accelerations below 0.02 m/s²
    #             if abs(ax_robot) < NOISE_THRESHOLD:
    #                 ax_robot = 0.0
    #             if abs(ay_robot) < NOISE_THRESHOLD:
    #                 ay_robot = 0.0
                    
    #             # Transform to world frame
    #             ax_world = ax_robot * math.cos(self.yaw) - ay_robot * math.sin(self.yaw)
    #             ay_world = ax_robot * math.sin(self.yaw) + ay_robot * math.cos(self.yaw)

    #             # BUG FIX #5: Apply velocity decay to prevent drift
    #             VELOCITY_DECAY = 0.995  # 0.5% decay per iteration
    #             self.vel_x *= VELOCITY_DECAY
    #             self.vel_y *= VELOCITY_DECAY
                
    #             # Integrate acceleration to velocity
    #             self.vel_x += ax_world * dt
    #             self.vel_y += ay_world * dt

    #             # BUG FIX #6: Velocity threshold (stop tiny drifts)
    #             VELOCITY_THRESHOLD = 0.001  # m/s
    #             if abs(self.vel_x) < VELOCITY_THRESHOLD:
    #                 self.vel_x = 0.0
    #             if abs(self.vel_y) < VELOCITY_THRESHOLD:
    #                 self.vel_y = 0.0

    #             # Integrate velocity to position
    #             self.imu_x += self.vel_x * dt
    #             self.imu_y += self.vel_y * dt

    #             # Store data for analysis
    #             self.imu_path.append((self.imu_x*100, self.imu_y*100))
    #             self.raw_accel_data.append((ax_world, ay_world, az_robot))

    #             # DEBUG: Print detailed info every 100 samples
    #             if sample_count % 100 == 0:
    #                 print(f"\n📊 IMU Debug Report (Sample {sample_count}):")
    #                 print(f"  ⏱️  dt: {dt*1000:.1f}ms (issues: {dt_issues})")
    #                 print(f"  📐 Yaw: {math.degrees(self.yaw):.1f}°")
    #                 print(f"  🏃 Raw accel: ({ax_robot:.4f}, {ay_robot:.4f}) m/s²")
    #                 print(f"  🌍 World accel: ({ax_world:.4f}, {ay_world:.4f}) m/s²") 
    #                 print(f"  💨 Velocity: ({self.vel_x:.4f}, {self.vel_y:.4f}) m/s")
    #                 print(f"  📍 Position: ({self.imu_x:.2f}, {self.imu_y:.2f}) m")
    #                 print(f"  📏 Position: ({self.imu_x*100:.1f}, {self.imu_y*100:.1f}) cm")
                    
    #                 # Check if we're actually moving
    #                 recent_accels = self.raw_accel_data[-10:] if len(self.raw_accel_data) >= 10 else self.raw_accel_data
    #                 if recent_accels:
    #                     avg_accel_mag = np.mean([math.sqrt(a[0]**2 + a[1]**2) for a in recent_accels])
    #                     print(f"  🔍 Recent accel magnitude: {avg_accel_mag:.4f} m/s²")
                        
    #                     if avg_accel_mag < 0.01:
    #                         print(f"  😴 Robot appears stationary")
    #                     elif avg_accel_mag > 0.1:
    #                         print(f"  🏃 Robot is moving!")
    #                     else:
    #                         print(f"  🤔 Unclear if moving")

    #             self.last_time = current_time
    #             time.sleep(0.01)

    #     except Exception as e:
    #         print(f"❌ IMU Error: {e}")
    #         import traceback
    #         traceback.print_exc()
    
    #     print(f"🏁 IMU thread ending. Total samples: {sample_count}, dt issues: {dt_issues}, accel readings: {accel_readings}")
    

    # # ALSO ADD THIS ENHANCED ANALYSIS METHOD:
    # def analyze_noise_and_reliability_debug(self):
    #     """Enhanced analysis with debugging info"""
        
    #     if len(self.raw_accel_data) < 10:
    #         print("❌ Insufficient IMU data for analysis")
    #         return

    #     accel_data = np.array(self.raw_accel_data)
        
    #     print(f"\n🔬 DETAILED ANALYSIS:")
    #     print(f"📊 Total IMU samples: {len(accel_data)}")
    #     print(f"📊 Total IMU path points: {len(self.imu_path)}")
        
    #     # Check for movement in IMU path
    #     if len(self.imu_path) > 1:
    #         start_pos = np.array(self.imu_path[0])
    #         end_pos = np.array(self.imu_path[-1])
    #         imu_movement = np.linalg.norm(end_pos - start_pos)
    #         print(f"📏 IMU total movement: {imu_movement:.1f} cm")
            
    #         if imu_movement < 5:  # Less than 5cm movement
    #             print(f"🚨 WARNING: IMU shows minimal movement! Possible integration issues.")
        
    #     # Analyze acceleration distribution
    #     ax_data = accel_data[:, 0]
    #     ay_data = accel_data[:, 1]
        
    #     print(f"📈 Accel X: min={np.min(ax_data):.4f}, max={np.max(ax_data):.4f}, range={np.max(ax_data)-np.min(ax_data):.4f}")
    #     print(f"📈 Accel Y: min={np.min(ay_data):.4f}, max={np.max(ay_data):.4f}, range={np.max(ay_data)-np.min(ay_data):.4f}")
        
    #     # Check for stuck values (common bug indicator)
    #     unique_x = len(np.unique(ax_data))
    #     unique_y = len(np.unique(ay_data))
    #     print(f"🔢 Unique X values: {unique_x}/{len(ax_data)} ({unique_x/len(ax_data)*100:.1f}%)")
    #     print(f"🔢 Unique Y values: {unique_y}/{len(ay_data)} ({unique_y/len(ay_data)*100:.1f}%)")
        
    #     if unique_x < len(ax_data) * 0.1 or unique_y < len(ay_data) * 0.1:
    #         print(f"🚨 WARNING: Very few unique acceleration values! Sensor may be stuck or integration broken.")
        
    #     # Rest of your original analysis
    #     ax_std = np.std(ax_data)
    #     ay_std = np.std(ay_data)
    #     ax_mean = np.mean(ax_data)
    #     ay_mean = np.mean(ay_data)

    #     self.noise_analysis = {
    #         'accel_x_std': ax_std,
    #         'accel_y_std': ay_std,
    #         'accel_x_mean': ax_mean,
    #         'accel_y_mean': ay_mean,
    #         'signal_to_noise_x': abs(ax_mean) / ax_std if ax_std > 0 else 0,
    #         'signal_to_noise_y': abs(ay_mean) / ay_std if ay_std > 0 else 0,
    #         'total_samples': len(accel_data),
    #         'imu_movement_cm': imu_movement if 'imu_movement' in locals() else 0
    #     }

    #     print('\n' + '='*50)
    #     print('📊 NOISE AND RELIABILITY ANALYSIS')
    #     print('='*50)
    #     print(f'🔢 Total samples: {self.noise_analysis["total_samples"]}')
    #     print(f'📈 Accel X: mean={ax_mean:.4f}, std={ax_std:.4f} m/s²')
    #     print(f'📈 Accel Y: mean={ay_mean:.4f}, std={ay_std:.4f} m/s²')
    #     print(f'📏 IMU movement: {self.noise_analysis["imu_movement_cm"]:.1f} cm')
    #     print(f'📡 Signal-to-noise ratio X: {self.noise_analysis["signal_to_noise_x"]:.2f}')
    #     print(f'📡 Signal-to-noise ratio Y: {self.noise_analysis["signal_to_noise_y"]:.2f}')
        
    #     # Enhanced recommendations
    #     self.provide_integration_recommendation_debug()

    # def provide_integration_recommendation_debug(self):
    #     """Enhanced recommendation with specific debugging"""
    #     snr_x = self.noise_analysis['signal_to_noise_x']
    #     snr_y = self.noise_analysis['signal_to_noise_y']
    #     movement = self.noise_analysis['imu_movement_cm']
        
    #     print(f"\n🤖 DETAILED DIAGNOSIS:")
        
    #     if movement < 5:
    #         print(f"❌ CRITICAL: IMU integration is BROKEN - no significant movement detected")
    #         print(f"💡 This is definitely a code bug, not sensor noise")
    #         return
        
    #     if snr_x < 2 or snr_y < 2:
    #         print("❌ DON'T INTEGRATE: Signal-to-noise ratio too low")
    #         if movement > 20:
    #             print("💡 But IMU did track movement - noise filtering might help")
    #         else:
    #             print("💡 Recommend debugging integration code first")
    #     else:
    #         print("✅ INTEGRATE: IMU shows good reliability!")



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


        plt.plot(np.array(x_coords),np.array(y_coords),'b-',linewidth=3,label="Robot Path - Encoder")

        imu_x_coords = [int(point[0]) for point in self.imu_path]
        imu_y_coords = [int(point[1]) for point in self.imu_path]

        plt.plot(x_coords[0],y_coords[0],'go',markersize=10,label="Start")
        plt.plot(x_coords[-1],y_coords[-1],'ro',markersize=10,label="End")


        plt.plot(np.array(x_coords),np.array(y_coords),'b-',linewidth=3,label="Robot Path - Encoder")


        plt.grid(True)
        plt.axis('equal')

        plt.title("Trajectories")

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
      
      robot_controller.forward_controller(60)

      time.sleep(1)

      print(f"Encoder odometry:{robot_controller.x,robot_controller.y}")
      print(f"IMU odometry:{robot_controller.imu_x,robot_controller.imu_y}")

      robot_controller.analyze_noise_and_reliability()
    #   robot_controller.turn_by_angle(90)
    #   robot_controller.forward_controller(60)
    #   robot_controller.turn_by_angle(-90)
    #   robot_controller.forward_controller(60)
    #   robot_controller.turn_by_angle(90)
    #   robot_controller.forward_controller(60)

    




    except KeyboardInterrupt:
        print("Stopping.....")
    finally:
        robot_controller.cleanup()
        robot_controller.noise_plot()
        # time.sleep(2)
        # robot_controller.publish_plot()

if __name__ == "__main__":
    main()