import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from roomba_interfaces.msg import SensorData, Odometry, ImuData

class OdomNode(Node):

    def __init__(self):
        super().__init__("kf_odom_node")

        self.angular_comp = None
        self.w_z = 0

        self.theta = 0

        self.total_LC = 0
        self.total_RC = 0

        self.last_left = 0
        self.last_right = 0
        self.first_reading = True

        self.have_imu = False
        self.enc_cnt = 0

        self.imu_subscriber = self.create_subscription(ImuData,"/imu",self.imu_callback,10)

        self.sensor_subscriber = self.create_subscription(SensorData,'/sensor_data',self.sensor_callback,10)

        
        #KF - INITIALIZATION

        self.xk = np.array([0,0,0,0,0,0])
        self.pk = np.array([[1e-4, 0, 0, 0, 0 ,0],
                   [0, 1e-4, 0, 0, 0 ,0],
                   [0, 0, 0.1, 0, 0 ,0],
                   [0, 0, 0, 0.1, 0 ,0],
                   [0, 0, 0, 0, 0.1 ,0],
                   [0, 0, 0, 0, 0 ,0.4]])
        
        self.zk = np.array([0,0,0])
        
        self.q = np.array([[0.01, 0, 0, 0, 0 ,0],
                   [0, 0.01, 0, 0, 0 ,0],
                   [0, 0, 0.02, 0, 0 ,0],
                   [0, 0, 0, 0.05, 0 ,0],
                   [0, 0, 0, 0, 0.2 ,0],
                   [0, 0, 0, 0, 0 ,0.5]])


        self.r = np.array([[0.01, 0, 0],
                  [0,0.03,0],
                  [0,0,0.02]])
        
        self.h = np.array([[0,0,0,1,0,0],
                  [0,0,0,0,1,0],
                  [0,0,1,0,0,0]])
        
        self.ppr = 11
        self.circumference = 0.1885  #m 

        self.last_time = time.time()

 
        self.get_logger().info("KF node has started........")
        
        self.timer_callback = self.create_timer(0.1,self.filter_callback)


    def F_k(self,px, py, theta, v, w_z, a, dt):
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array([
            [1.0, 0.0, -(v*s*dt) - 0.5*a*s*dt*dt,  c*dt, 0.0, 0.5*c*dt*dt],
            [0.0, 1.0,  (v*c*dt) + 0.5*a*c*dt*dt,  s*dt, 0.0, 0.5*s*dt*dt],
            [0.0, 0.0,  1.0,                        0.0,  dt,  0.0],
            [0.0, 0.0,  0.0,                        1.0,  0.0, dt],
            [0.0, 0.0,  0.0,                        0.0,  1.0, 0.0],
            [0.0, 0.0,  0.0,                        0.0,  0.0, 1.0],
        ])


    def imu_callback(self,msg):
        if self.angular_comp == None:
            self.angular_comp = msg.w_z
        self.w_z = msg.w_z - self.angular_comp
        
        self.theta = msg.yaw
        self.have_imu = True

    def sensor_callback(self,msg):
        if (self.first_reading):
            self.last_left = msg.left_encoder
            self.last_right = msg.right_encoder
            self.first_reading = False
            self.enc_cnt = 1
            return

        self.total_LC = msg.left_encoder
        self.total_RC = msg.right_encoder
        self.enc_cnt += 1


    def distance_covered(self,LC,RC):
        left_distance = (self.circumference/self.ppr)*LC
        right_distance = (self.circumference/self.ppr)*RC

        return (left_distance + right_distance)/2

    def filter_callback(self):

        if not self.have_imu and self.enc_cnt < 2:
            return
        
        

        curr_time = time.time()
        dt = curr_time - self.last_time
        self.last_time = curr_time

        delta_lc = self.total_LC - self.last_left
        delta_rc = self.total_RC - self.last_right


        v_odom = (self.distance_covered(delta_lc,delta_rc))/dt

        self.zk = np.array([v_odom,float(self.w_z),float(self.theta)],dtype=float)

        px,py,theta,v,w_z,a = self.xk

        self.xk = np.array([
            px + v*math.cos(theta)*dt + 1/2*(a*math.cos(theta)*dt**2),
            py + v*math.sin(theta)*dt + 1/2*(a*math.sin(theta)*dt**2),
            theta + w_z*dt,
            v + a*dt,
            w_z,
            a
        ],dtype=float)

        f_k = self.F_k(px, py, theta, v, w_z, a, dt)

        self.pk = f_k.dot(self.pk).dot(f_k.T) + self.q

        measurement_residual = self.zk - self.h.dot(self.xk)

        residual_covariance = self.h.dot(self.pk).dot(self.h.T) + self.r

        k_k = (self.pk).dot(self.h.T).dot(np.linalg.inv(residual_covariance))

        self.xk = self.xk + k_k.dot(measurement_residual)

        self.pk = (np.eye(len(self.xk)) - k_k.dot(self.h)).dot(self.pk)

        self.last_left = self.total_LC
        self.last_right = self.total_RC

        self.get_logger().info(f'X-Coord:{self.xk[0]*100:.3f}, Y-Coord:{self.xk[1]*100:.3f}')

        


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    



if __name__ == "main":
    main()