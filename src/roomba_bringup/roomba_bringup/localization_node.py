import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from roomba_interfaces.msg import SensorData
from nav_msgs.msg import OccupancyGrid
from roomba_interfaces.msg import LocalizationData
import numpy as np
import math


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("loc_node")
        
        
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.imu_subscriber = self.create_subscription(Float32,'/imu',self.imu_callback,10)
        self.sensor_subscriber = self.create_subscription(SensorData,'/wheel_states',self.sensor_callback,10)
        
        self.loc_publisher = self.create_publisher(LocalizationData,'/loc',10)
        
        self.map_arr = None
        
        self.cell_size = 22 ### In cm
        self.height = 300
        self.width = 200
        
        self.height_cells = 14
        self.width_cells = 9

        
        self.cum_left = 0.0
        self.cum_right = 0.0
      
        
        self.last_left = 0.0
        self.last_right = 0.0
    
        #Initial position is physically known, since that's the same place we started for mapping
        self.x = 0.0        
        self.y = 0.0
        self.theta = 0.0
        
        #Placing at (7,4) in this case since that's where mapping also started
        self.grid_x = 7
        self.grid_y = 4
        
        self.confidence = 95
        
        self.actual_obstacle_dist = 0.0
        self.predicted_obstacle_dist = 0.0
        
        self.cum_jump = 5 #cm
        
        self.timer = self.create_timer(1,self.localization_callback)
        
        self.match_score  = None

        


        self.get_logger().info('🧭 Localization Node Started!')
        self.get_logger().info(f'📍 Initial position: grid=({self.grid_x},{self.grid_y}), confidence={self.confidence}%')
        
    def imu_callback(self,msg):
        self.theta = msg.data
        
        
    def map_callback(self,msg):
        width = msg.info.width
        height = msg.info.height
        
        self.map_arr = np.array(msg.data).reshape((height,width))
        
        
    def sensor_callback(self,msg):
        self.cum_left, self.cum_right, self.actual_obstacle_dist = msg.left_encoder, msg.right_encoder,msg.ultrasonic_distance
        
    def is_valid_coord(self,x,y):
        return (-150<=x<=150 and -100<=y<=100)
        
    def is_valid_grid(self,grid_x,grid_y):
        return (0<=grid_x<self.height_cells and 0<=grid_y<self.width_cells)  
        
    def localization_callback(self):
        
        #Step 1 - Motion Prediction
        d_left = self.cum_left - self.last_left
        d_right = self.cum_right - self.last_right
        
        d = (d_left + d_right)/2
        
        new_x = self.x
        new_y = self.y
        
        new_x += d*math.cos(self.theta) # Update only when it's a valid coord , else clamp it
        new_y += d*math.sin(self.theta)
        
        if(self.is_valid_coord(new_x,new_y)):
            self.x = new_x
            self.y = new_y
            
        else:
            self.x = max(-150,min(150,new_x))
            self.y = max(-100,min(100,new_y))
            
        
        # Updating the GRID coordinates
        del_gridx = d*math.cos(self.theta)/self.cell_size
        del_gridy = d*math.sin(self.theta)/self.cell_size
        
        new_grid_x = self.grid_x + round(del_gridy)
        new_grid_y = self.grid_y + round(del_gridx)
        
        if(self.is_valid_grid(new_grid_x,new_grid_y)):
            self.grid_x = new_grid_x
            self.grid_y = new_grid_y
            

            
            
        else:
            self.grid_x = max(0,min(self.height_cells-1,new_grid_x))
            self.grid_y = max(0,min(self.width_cells-1,new_grid_y))
            

        old_confidence = self.confidence
        self.confidence = self.confidence * 0.9 
        
        #Step2 - Sensor Simulation
        self.predicted_obstacle_dist = self.sensor_sim_func()
        
        #Step3 - Position Verification
        numerator = min(self.predicted_obstacle_dist,self.actual_obstacle_dist)
        denominator = max(self.predicted_obstacle_dist,self.actual_obstacle_dist)
        self.match_score = round(numerator/denominator * 100)
        
        #Step5 - Confidence Update
        new_confidence = self.confidence
        if self.match_score > 80:
            new_confidence += 10
            self.get_logger().info('✅ Good sensor match - confidence increased')
        elif self.match_score < 50:
            new_confidence -= 20
            self.get_logger().info('❌ Poor sensor match - confidence decreased')
        else:
            self.get_logger().info('➖ Average sensor match - confidence unchanged')
            
        self.confidence = max(0,min(95,new_confidence))

        self.get_logger().info(f'📈 Confidence: {old_confidence:.1f}% → {self.confidence:.1f}%')

        self.last_left = self.cum_left
        self.last_right = self.cum_right
        
        #Step6 - Final Step
        if (self.confidence > 70):
            self.get_logger().info("Confident about my location")
        elif (30<=self.confidence<70):
            self.get_logger().info("Not sure about my location")
        else:
            self.get_logger().info("I'm Lost :) ")

        self.get_logger().info(f'🏁 Final position: grid=({self.grid_x},{self.grid_y}), confidence={self.confidence:.1f}%')

        self.publish_loc_data()
        self.get_logger().info('🧭 ===== END LOCALIZATION CYCLE =====\n')
        

    def publish_loc_data(self):
        loc_msg = LocalizationData()
        loc_msg.x = self.x
        loc_msg.y = self.y
        loc_msg.theta = self.theta
        loc_msg.grid_x = self.grid_x
        loc_msg.grid_y = self.grid_y
        loc_msg.confidence = self.confidence
      
        
        self.loc_publisher.publish(loc_msg)
        
        self.get_logger().debug(f'📍 Published Localization Data: {loc_msg}')
        
    def sensor_sim_func(self):
        # Using local variable instead of instance variable
        current_distance = 0
        step_size = 5  # cm
        max_range = 200  # cm
        
        while current_distance < max_range:
    
            
            # Convert world to grid 
            grid_x = self.grid_x + round((current_distance * math.sin(self.theta)) / self.cell_size)
            grid_y = self.grid_y + round((current_distance * math.cos(self.theta)) / self.cell_size)
            
            self.get_logger().debug(f'🔍 Testing distance {current_distance}cm → grid ({grid_x},{grid_y})')
            
            if(self.is_valid_grid(grid_x, grid_y)):
                if (self.map_arr[grid_x, grid_y] == 100):
                    self.get_logger().info(f'🚧 Found obstacle at distance {current_distance}cm, grid ({grid_x},{grid_y})')
                    return current_distance
                else:
                    current_distance += step_size
            else:
                self.get_logger().info(f'🌐 Hit map boundary at distance {current_distance}cm')
                return current_distance  # Hit boundary
                
        self.get_logger().info(f'👀 No obstacle found within {max_range}cm range')
        return max_range  # No obstacle found
        
        
        

def main(args=None):
    rclpy.init(args=args)
    loc_node = LocalizationNode()
    rclpy.spin(loc_node)
    loc_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()