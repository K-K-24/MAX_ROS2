import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from roomba_interfaces.msg import SensorData
from nav_msgs.msg import OccupancyGrid
from roomba_interfaces.msg import LocalizationData
import numpy as np
import math
import json
import os

class LocalizationNode(Node):
    def __init__(self):
        super().__init__("loc_node")
        
        # Map loading configuration
        self.map_directory = os.path.expanduser('~/ros2_ws/maps/')
        self.use_saved_map = True  # Set to True to load from file, False to use live map
        
        # Subscribe to live map (backup)
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.imu_subscriber = self.create_subscription(Float32,'/imu',self.imu_callback,10)
        self.sensor_subscriber = self.create_subscription(SensorData,'/wheel_states',self.sensor_callback,10)
        
        self.loc_publisher = self.create_publisher(LocalizationData,'/loc',10)
        
        self.map_arr = None
        self.map_metadata = None
        
        self.cell_size = 22 ### In cm
        self.height = 100
        self.width = 100
        
        self.height_cells = 5
        self.width_cells = 5

        self.cum_left = float(0.0)
        self.cum_right = float(0.0)
        
        self.last_left = float(0.0)
        self.last_right = float(0.0)
    
        #Initial position is physically known, since that's the same place we started for mapping
        self.x = float(0.0)        
        self.y = float(0.0)
        self.theta = float(0.0)
        
        #Placing at (7,4) in this case since that's where mapping also started
        self.grid_x = int(1)
        self.grid_y = int(1)

        self.sum_grid_x = int(self.grid_x)
        self.sum_grid_y = int(self.grid_y)
        
        self.confidence = float(95.0)
        
        self.actual_obstacle_dist = float(0.0)
        self.predicted_obstacle_dist = float(0.0)
        
        self.timer = self.create_timer(1, self.localization_callback)
        
        self.match_score = None

        # Load saved map if available
        if self.use_saved_map:
            self.load_map_from_file()
        
        self.get_logger().info('🧭 Localization Node Started!')
        self.get_logger().info(f'📍 Initial position: grid=({self.grid_x},{self.grid_y}), confidence={self.confidence}%')
        
        if self.use_saved_map:
            if self.map_arr is not None:
                self.get_logger().info('📁 Using saved map from file')
            else:
                self.get_logger().warn('⚠️  Failed to load saved map, will use live map')
        else:
            self.get_logger().info('📡 Using live map from mapper node')

    def load_map_from_file(self, filename='latest_map.json'):
        """Load map from saved JSON file"""
        try:
            filepath = os.path.join(self.map_directory, filename)
            
            if not os.path.exists(filepath):
                self.get_logger().warn(f'📁 Map file not found: {filepath}')
                self.get_logger().info('📡 Will fall back to live map')
                return False
            
            with open(filepath, 'r') as f:
                map_data = json.load(f)
            
            # Load map array
            self.map_arr = np.array(map_data['map_array'], dtype=np.int8)
            self.map_metadata = map_data['metadata']
            
            # Update parameters from saved map
            self.height_cells = self.map_metadata['height_cells']
            self.width_cells = self.map_metadata['width_cells'] 
            self.cell_size = self.map_metadata['cell_size_cm']
            
            # Update initial position to match mapping start
            # self.grid_x = self.map_metadata['starting_grid_x']
            # self.grid_y = self.map_metadata['starting_grid_y']
            
            # Log map statistics
            stats = map_data['statistics']
            self.get_logger().info(f'✅ Map loaded successfully from {filename}')
            self.get_logger().info(f'📊 Map stats: {stats["free_cells"]} free, {stats["obstacle_cells"]} obstacles, {stats["unknown_cells"]} unknown')
            self.get_logger().info(f'📐 Map size: {self.height_cells}x{self.width_cells} cells, {self.cell_size}cm per cell')
            self.get_logger().info(f'🎯 Starting position: ({self.grid_x}, {self.grid_y})')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load map from file: {e}')
            return False

    def list_available_maps(self):
        """List all available map files"""
        try:
            if not os.path.exists(self.map_directory):
                self.get_logger().warn(f'📁 Map directory does not exist: {self.map_directory}')
                return []
            
            map_files = [f for f in os.listdir(self.map_directory) if f.endswith('.json')]
            map_files.sort(reverse=True)  # Most recent first
            
            self.get_logger().info(f'📋 Available maps in {self.map_directory}:')
            for i, filename in enumerate(map_files):
                self.get_logger().info(f'  {i+1}. {filename}')
                
            return map_files
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to list maps: {e}')
            return []

    def imu_callback(self, msg):
        self.theta = float(msg.data)
        
    def map_callback(self, msg):
        """Callback for live map (backup if saved map not available)"""
        if self.map_arr is None:  # Only use live map if no saved map loaded
            width = msg.info.width
            height = msg.info.height
            
            self.map_arr = np.array(msg.data).reshape((height, width))
            
            self.get_logger().info(f'✅ Live map received! Size: {height}x{width}')
        
    def sensor_callback(self, msg):
        # 🛡️ TYPE SAFETY: Ensure all values are proper Python types
        self.cum_left = float(msg.left_encoder)
        self.cum_right = float(msg.right_encoder) 
        self.actual_obstacle_dist = float(msg.ultrasonic_distance)
        
    def is_valid_coord(self, x, y):
        return (-55 <= x <= 55 and -55 <= y <= 55)
        
    def is_valid_grid(self, grid_x, grid_y):
        return (0 <= grid_x < self.height_cells and 0 <= grid_y < self.width_cells)  
        
    def localization_callback(self):
        # 🛡️ SAFETY CHECK: Don't run localization without map data
        if self.map_arr is None:
            self.get_logger().warn('⚠️  No map data available yet, skipping localization cycle')
            return

        self.get_logger().info(f'\n🔄 ===== LOCALIZATION CYCLE =====')
        
        #Step 1 - Motion Prediction
        d_left = float(self.cum_left - self.last_left)
        d_right = float(self.cum_right - self.last_right)
        
        d = float((d_left + d_right) / 2)
        
        new_x = float(self.x)
        new_y = float(self.y)
        
        new_x += float(d * math.cos(self.theta))
        new_y += float(d * math.sin(self.theta))
        
        if self.is_valid_coord(new_x, new_y):
            self.x = float(new_x)
            self.y = float(new_y)
        else:
            self.get_logger().warn(f'🚫 Invalid coordinates: ({new_x:.2f}, {new_y:.2f}) - Clamping to valid range')
            self.x = float(max(-150, min(150, new_x)))
            self.y = float(max(-100, min(100, new_y)))
        
        # Updating the GRID coordinates
        del_gridx = float(d * math.cos(self.theta) / self.cell_size)
        del_gridy = float(d * math.sin(self.theta) / self.cell_size)

        self.sum_grid_x += del_gridx
        self.sum_grid_y += del_gridy
        
        new_grid_x = round(self.sum_grid_x)
        new_grid_y = round(self.sum_grid_y)
        
        if self.is_valid_grid(new_grid_x, new_grid_y):
            self.grid_x = int(new_grid_x)
            self.grid_y = int(new_grid_y)
        else:
            self.get_logger().warn(f'🚫 Invalid grid position: ({new_grid_x}, {new_grid_y}) - Clamping to valid range')
            self.grid_x = int(max(0, min(self.height_cells-1, new_grid_x)))
            self.grid_y = int(max(0, min(self.width_cells-1, new_grid_y)))

        old_confidence = float(self.confidence)

        if d > 0:
            self.confidence = float(self.confidence * 0.9)
            
            #Step2 - Sensor Simulation
            self.predicted_obstacle_dist = float(self.sensor_sim_func(new_x,new_y))
            
            #Step3 - Position Verification
            numerator = float(min(self.predicted_obstacle_dist, self.actual_obstacle_dist))
            denominator = float(max(self.predicted_obstacle_dist, self.actual_obstacle_dist))
            
            if denominator > 0:
                self.match_score = int(round(numerator / denominator * 100))
            else:
                self.match_score = 0

            self.get_logger().info(f'🔍 Predicted distance: {self.predicted_obstacle_dist:.1f}cm, Actual distance: {self.actual_obstacle_dist:.1f}cm,match_score: {self.match_score}')
            
            #Step5 - Confidence Update
            new_confidence = float(self.confidence)
            if self.match_score > 80:
                new_confidence += 10.0
                self.get_logger().info('✅ Good sensor match - confidence increased')
            elif self.match_score < 50:
                new_confidence -= 10.0
                self.get_logger().info('❌ Poor sensor match - confidence decreased')
            else:
                self.get_logger().info('➖ Average sensor match - confidence unchanged')
                
            self.confidence = float(max(0.0, min(95.0, new_confidence)))

        self.get_logger().info(f'📈 Confidence: {old_confidence:.1f}% → {self.confidence:.1f}%')
        self.get_logger().info(f'🎯 Match score: {self.match_score}% (predicted: {self.predicted_obstacle_dist:.1f}cm, actual: {self.actual_obstacle_dist:.1f}cm)')

        self.last_left = float(self.cum_left)
        self.last_right = float(self.cum_right)
        
        #Step6 - Final Step
        if self.confidence > 70:
            self.get_logger().info("😊 Confident about my location")
        elif 30 <= self.confidence < 70:
            self.get_logger().info("🤔 Not sure about my location")
        else:
            self.get_logger().info("😵 I'm Lost!")

        self.get_logger().info(f'🏁 Final position: grid=({self.grid_x},{self.grid_y}), confidence={self.confidence:.1f}%')

        self.publish_loc_data()
        self.get_logger().info('🧭 ===== END LOCALIZATION CYCLE =====\n')
        

    def publish_loc_data(self):
        loc_msg = LocalizationData()
        
        # 🛡️ TYPE SAFETY: Explicitly cast all values to ensure ROS2 compatibility
        loc_msg.x = float(self.x)
        loc_msg.y = float(self.y)
        loc_msg.theta = float(self.theta)
        loc_msg.grid_x = int(self.grid_x)
        loc_msg.grid_y = int(self.grid_y)
        loc_msg.confidence = float(self.confidence)
        
        self.loc_publisher.publish(loc_msg)
        
        self.get_logger().info(f'📍 Published Localization Data: x={loc_msg.x:.2f}, y={loc_msg.y:.2f}, grid=({loc_msg.grid_x},{loc_msg.grid_y}), conf={loc_msg.confidence:.1f}%')
        
    def sensor_sim_func(self, x, y):
        # 🛡️ SAFETY CHECK: Don't run sensor simulation without map data
        if self.map_arr is None:
            self.get_logger().warn('⚠️  No map data available for sensor simulation')
            return float(200.0)  # Return max range as fallback
        
        # Using local variable instead of instance variable
        current_distance = 0
        step_size = 5  # cm
        max_range = 200  # cm
        
        while current_distance < max_range:

            new_x = x + (current_distance * math.cos(self.theta))
            new_y = y + (current_distance * math.sin(self.theta))

            dist_x = new_x - x 
            dist_y = new_y - y

            grid_x = self.grid_x + round(dist_x / self.cell_size)
            grid_y = self.grid_y + round(dist_y / self.cell_size)

            self.get_logger().debug(f'🔍 Testing distance {current_distance}cm → grid ({grid_x},{grid_y})')
            
            if self.is_valid_grid(grid_x, grid_y):
                # ✅ SAFE: map_arr is guaranteed to exist
                if self.map_arr[grid_x, grid_y] == 100:
                    self.get_logger().info(f'🚧 Found obstacle at distance {current_distance}cm, grid ({grid_x},{grid_y})')
                    return float(current_distance)
                else:
                    current_distance += step_size
            else:
                self.get_logger().info(f'🌐 Hit map boundary at distance {current_distance}cm')
                return float(current_distance)  # Hit boundary
                
        self.get_logger().info(f'👀 No obstacle found within {max_range}cm range')
        return float(max_range)  # No obstacle found

def main(args=None):
    rclpy.init(args=args)
    loc_node = LocalizationNode()
    
    try:
        rclpy.spin(loc_node)
    except KeyboardInterrupt:
        loc_node.get_logger().info('🛑 Keyboard interrupt received')
    finally:
        loc_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()