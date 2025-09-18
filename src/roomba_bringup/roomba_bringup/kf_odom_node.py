import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from roomba_interfaces.msg import SensorData, ImuData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

class OdomNode(Node):

    def __init__(self):
        super().__init__("kf_odom_node")

        self.nav_odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.angular_comp = None
        self.w_z = 0

        self.theta = 0

        self.total_LC = 0
        self.total_RC = 0

        self.last_left = 0
        self.last_right = 0
        self.first_reading = True

        self.x = 0.0
        self.y = 0.0

        self.x_odom = []
        self.y_odom = []

        self.x_kf_odom = []
        self.y_kf_odom = []

        self.residuals = []

        self.have_imu = False
        self.enc_cnt = 0

        self.imu_subscriber = self.create_subscription(ImuData,"/imu",self.imu_callback,10)

        self.sensor_subscriber = self.create_subscription(SensorData,'/wheel_states',self.sensor_callback,10)

        
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


        self.r = np.array([[0.15, 0, 0],
                  [0,0.03,0],
                  [0,0,0.05]])
        
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

        if not self.have_imu or self.enc_cnt < 2:
            return
        
        

        curr_time = time.time()
        dt = curr_time - self.last_time
        if ( dt <= 0):
            return
        self.last_time = curr_time

        delta_lc = self.total_LC - self.last_left
        delta_rc = self.total_RC - self.last_right

        d = self.distance_covered(delta_lc,delta_rc)

        self.x += d*math.cos(self.theta)
        self.y += d*math.sin(self.theta)

        self.x_odom.append(self.x*100)
        self.y_odom.append(self.y*100)


        v_odom = d/dt

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

        self.residuals.append(measurement_residual)


        residual_covariance = self.h.dot(self.pk).dot(self.h.T) + self.r

        k_k = (self.pk).dot(self.h.T).dot(np.linalg.inv(residual_covariance))

        self.xk = self.xk + k_k.dot(measurement_residual)

        self.pk = (np.eye(len(self.xk)) - k_k.dot(self.h)).dot(self.pk)

        self.last_left = self.total_LC
        self.last_right = self.total_RC

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child.frame_id = "base_link"

        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]
        odom.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(0.0,0.0,self.theta)
        odom.pose.pose.orientation = quat

        self.nav_odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.xk[0]
        t.transform.translation.y = self.xk[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)

        self.x_kf_odom.append(self.xk[0]*100)
        self.y_kf_odom.append(self.xk[1]*100)

        self.get_logger().info(f'X-Coord:{self.xk[0]*100:.3f}, Y-Coord:{self.xk[1]*100:.3f}')

    def analyze(self,residuals,name):
        mean = np.mean(residuals)
        std = np.std(residuals)
        print(f"{name}: mean = {mean:.4f}, std={std:.4f}")
        plt.hist(residuals,bins=40,alpha=0.6,label=name,histtype='step',linewidth=2)

    def publish_residual_plot(self):
        res = np.array(self.residuals)
        res_v,res_w,res_theta = res[:,0],res[:,1],res[:,2]

        plt.figure(figsize=(10,5))

        self.analyze(res_v, "v(odom)")
        self.analyze(res_w,"wz (imu)")
        self.analyze(res_theta,"theta (imu)")

        plt.legend()
        plt.title("Residual Histograms")
        plt.xlabel("Residual Value")
        plt.ylabel("Frequency")
        plt.tight_layout()
        plt.savefig("residuals_hist.png",dpi=200)
        plt.close()

    def euler_to_quaternion(self,roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return Quaternion(x=qx, y=qy, z=qz, w=qw)



    def publish_plot(self):

        plt.figure()
        plt.plot(0,0,'go',markersize=10,label="Start")


        plt.plot(np.array(self.x_odom),np.array(self.y_odom),'b-',linewidth=3,label="Odometry")
        plt.plot(np.array(self.x_kf_odom),np.array(self.y_kf_odom),'r-',linewidth=3,label="KF-Odometry")

        plt.grid(True)
        plt.axis('equal')
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.title("Trajectory: Odometry vs KF-Odometry")
        plt.legend()
        plt.tight_layout()
        plt.savefig("trajectory_compare.png",dpi=200)
        plt.close()



        


def main(args=None):
                
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.publish_plot()
        node.publish_residual_plot()
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()