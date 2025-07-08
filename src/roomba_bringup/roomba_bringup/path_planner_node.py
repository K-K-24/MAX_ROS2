import rclpy
from rclpy.node import Node
from roomba_interfaces.msg import LocalizationData
from nav_msgs.msg import OccupancyGrid
from roomba_interfaces.msg import SensorData
from geometry_msgs.msg import TwistStamped
import numpy as np
import threading
import math
import time
import os
import json
import matplotlib.pyplot as plt

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.get_logger().info('🤖 Path Planner Node Started')

        self.sensor_sub = self.create_subscription(SensorData, '/wheel_states', self.sensor_callback, 10)
        
        self.loc_subscriber = self.create_subscription(LocalizationData,"/loc",self.loc_callback,10)
        # self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.vel_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel',10)

        self.map_directory = os.path.expanduser('~/ros2_ws/maps/')

        self.map_metadata = None

        self.linear_vel = float(0.0)
        self.angular_vel = float(0.0)
    
        self.grid_x = int(1)
        self.grid_y = int(1)
        
        self.goal_grid_x = int(0)
        self.goal_grid_y = int(0)
        
        self.x = float(0.0)
        self.y = float(0.0)
        self.theta = float(0.0)
        
        self.cell_side = int(22)
        
        self.confidence = float(0.0)
        
        self.map_arr = None
        
        self.height = int(0)
        self.width = int(0)

        self.obstacle_dist = float(0.0)  # Distance from the nearest obstacle
        
        self.queue = []
        self.visited = []
        self.parent = {}
        self.goal = None
        
        self.paths = []    # Has only one path as of now
        self.optimized_path = None

        #Plotting variables
        self.actual_path = []
        self.planned_path_coords = []
        self.turning_points = []

        self.load_map_from_file()  # Load map from file if available

        self.get_logger().info('✅ Path Planner initialized and ready')

    def sensor_callback(self, msg):
        self.obstacle_dist = msg.ultrasonic_distance
        
    def loc_callback(self,msg):
        # 🛡️ TYPE SAFETY: Ensure proper types
        self.grid_x = int(msg.grid_x)
        self.grid_y = int(msg.grid_y)
        self.theta = float(msg.theta)
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.confidence = float(msg.confidence)

        self.actual_path.append((self.x, self.y))

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
            self.height = self.map_metadata['height_cells']
            self.width = self.map_metadata['width_cells'] 
          
            
            # Update initial position to match mapping start
            # self.grid_x = self.map_metadata['starting_grid_x']
            # self.grid_y = self.map_metadata['starting_grid_y']
            
            # Log map statistics
            stats = map_data['statistics']
            self.get_logger().info(f'✅ Map loaded successfully from {filename}')
            self.get_logger().info(f'📊 Map stats: {stats["free_cells"]} free, {stats["obstacle_cells"]} obstacles, {stats["unknown_cells"]} unknown')
            self.get_logger().info(f'🎯 Starting position: ({self.grid_x}, {self.grid_y})')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load map from file: {e}')
            return False
        
    # def map_callback(self,msg):
    #     self.height = int(msg.info.height)
    #     self.width = int(msg.info.width)
    #     self.map_arr = np.array(msg.data).reshape((self.height,self.width))
        
    # As of now only one path is taken( Cost function is not taken into account)
    def path_planner(self): 
            # Input validation logging
        self.get_logger().info(f'🎯 Starting pathfinding from ({self.grid_x},{self.grid_y}) to ({self.goal_grid_x},{self.goal_grid_y})')

        if self.map_arr is None:
            self.get_logger().error('❌ No map received yet!')
            return

            # Goal validation
        if not (0 <= self.goal_grid_x < self.height and 0 <= self.goal_grid_y < self.width):
            self.get_logger().error(f'❌ Goal ({self.goal_grid_x},{self.goal_grid_y}) is outside map bounds ({self.height}x{self.width})')
            return "Goal out of bounds"
        
        if not self.is_free([self.goal_grid_x, self.goal_grid_y]):
            self.get_logger().error(f'❌ Goal ({self.goal_grid_x},{self.goal_grid_y}) is occupied!')
            return "Goal is occupied"

        self.queue = []
        self.visited = []
        self.parent = {}


        self.queue.append([self.grid_x, self.grid_y])
        self.visited.append([self.grid_x, self.grid_y])
        self.goal = [self.goal_grid_x,self.goal_grid_y]

        nodes_explored = 0


        
        while self.queue :
            current = self.queue.pop(0)
            nodes_explored += 1
            
            if(current == self.goal ):
                self.get_logger().info(f'✅ Path found! Explored {nodes_explored} nodes')
                self.paths = []  # Reset paths for new search
                self.paths.append(self.reconstruct_path(self.parent,self.goal))
                self.optimized_path = self.paths[0]  # As of now, just taking as it is since there's only one path
                self.get_logger().info(f'🗺️  Optimized path: {self.optimized_path}')

                #Plotting
                self.convert_path_to_word_coords()


                self.execute_path()

                self.get_logger().info(f'📏 Path length: {len(self.optimized_path)} waypoints')
                self.get_logger().info(f'🛤️  Path: {self.optimized_path}')
                
                return "Path found and executing"
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in self.visited and self.is_free(neighbor):
                    self.visited.append(neighbor)
                    self.parent[tuple(neighbor)] = tuple(current)
                    self.queue.append(neighbor)
                    
        self.get_logger().error(f'❌ No path found after exploring {nodes_explored} nodes')
        return "No Path Found"
    
    def convert_path_to_word_coords(self):
        self.planned_path_coords = []

        origin_grid_x = 1
        origin_grid_y = 1

        for grid_point in self.optimized_path:
            grid_x = grid_point[0]
            grid_y = grid_point[1]

            # Convert grid coordinates to world coordinates
            world_x = (grid_x - origin_grid_x) * self.cell_side
            world_y = (grid_y - origin_grid_y) * self.cell_side
            
            self.planned_path_coords.append((world_x, world_y))

        self.get_logger().info(f'📍 Planned path coordinates: {self.planned_path_coords}')
        
    def execute_path(self):
        if (self.confidence > 10):
            self.get_logger().info(f'🚀 Starting path execution with confidence {self.confidence:.1f}%')
            
            #Step - 1 ( Dealing with theta difference)

            for i in range(1, len(self.optimized_path)):
            
                next_grid_x = self.optimized_path[i][0]
                next_grid_y = self.optimized_path[i][1] 
                
                # sub_goal_y = (next_grid_x - self.grid_x)*self.cell_side + self.y
                # sub_goal_x = (next_grid_y - self.grid_y)*self.cell_side + self.x
                
                desired_heading = float(math.atan2((next_grid_y - self.grid_y),(next_grid_x - self.grid_x)))           

                #Normalizing the angle to be between -π and π
                current_heading = self.normalize_angle(self.theta)
                desired_heading = self.normalize_angle(desired_heading)
                
                difference = self.normalize_angle(desired_heading - current_heading)

                self.get_logger().info(f'🧭 Current heading: {math.degrees(current_heading):.1f}°')
                self.get_logger().info(f'🧭 Target heading: {math.degrees(desired_heading):.1f}°')
                self.get_logger().info(f'🧭 Difference: {math.degrees(difference):.1f}°')

                # Skipping the no turn, and very minute difference for now
                if abs(difference) > 0.1:
                    if(difference > 0):
                        self.left_turn(desired_heading)
                    else:
                        self.right_turn(desired_heading)
                else:
                    self.get_logger().info('🛑 No significant turn needed, proceeding straight')
                    
                self.angular_vel = float(0) #For a good practise, setting the angular velocity back to 0
                time.sleep(0.5)  # Short pause to stabilize before moving straight
                self.get_logger().info('✅ Turn complete')

                rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update obstacle distance

                #Dealing with straight movement
                if (self.obstacle_dist < self.cell_side):
                    self.get_logger().warn(f'⚠️  Obstacle detected at {self.obstacle_dist:.1f}cm - stopping for safety')
                    self.obstacle_avoidance()  # Handle obstacle avoidance
                    self.path_planner()  # Restart path planning
                    return
                else:
                    self.get_logger().info(f'✅ No obstacles detected, proceeding to next waypoint ({next_grid_x},{next_grid_y})')  
                    self.straight_movement()

            self.create_path_plot()  # Create path plot after execution
                    
                
            
        else:
            self.get_logger().warn(f'⚠️  Low confidence ({self.confidence:.1f}%) - stopping for safety')

        if hasattr(self, 'completion_event'):
            self.completion_event.set() #Signalling completion


    def create_path_plot(self):
        """Create a simple plot showing planned vs actual path"""
        try:
            plt.figure(figsize=(10, 8))
            
            # Plot the map as background
            if self.map_arr is not None:
                # Convert map to plot coordinates
                plt.imshow(self.map_arr, cmap='gray_r', origin='lower', 
                          extent=[-self.width*self.cell_side//2, self.width*self.cell_side//2,
                                 -self.height*self.cell_side//2, self.height*self.cell_side//2])
            
            # Plot planned path
            if self.planned_path_coords:
                planned_x = [point[0] for point in self.planned_path_coords]
                planned_y = [point[1] for point in self.planned_path_coords]
                plt.plot(planned_x, planned_y, 'b-', linewidth=3, label='Planned Path', marker='o', markersize=8)
            
            # Plot actual path (last 100 points to avoid clutter)
            if len(self.actual_path) > 5:
                actual_x = [point[0] for point in self.actual_path[-100:]]
                actual_y = [point[1] for point in self.actual_path[-100:]]
                plt.plot(actual_x, actual_y, 'r.', markersize=2, label='Actual Path', alpha=0.7)
            
            # Plot turning points
            if self.turning_points:
                turn_x = [point[0] for point in self.turning_points]
                turn_y = [point[1] for point in self.turning_points]
                plt.scatter(turn_x, turn_y, c='orange', s=100, marker='*', label='Turning Points', zorder=5)
            
            # Plot start and goal
            if self.planned_path_coords:
                start_x, start_y = self.planned_path_coords[0]
                goal_x, goal_y = self.planned_path_coords[-1]
                plt.scatter(start_x, start_y, c='green', s=200, marker='s', label='Start', zorder=6)
                plt.scatter(goal_x, goal_y, c='red', s=200, marker='X', label='Goal', zorder=6)
            
            plt.xlabel('X (cm)')
            plt.ylabel('Y (cm)')
            plt.title(f'Robot Path Planning - Planned vs Actual')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            
            # Save plot
            plot_filename = f'robot_path_{int(time.time())}.png'
            plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
            plt.close()  # Close to save memory
            
            self.get_logger().info(f'📊 Plot saved as: {plot_filename}')
            
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to create plot: {e}')


    def straight_movement(self):
        # Step - 2 (Dealing with straight movement)
        distance_to_travel = float(self.cell_side)

        overshoot_compensation = 8  # 10% overshoot compensation
        actual_target = distance_to_travel - overshoot_compensation

        self.get_logger().info(f'📏 Moving forward {distance_to_travel:.1f}cm, , Stop at: {actual_target:.1f}cm')
        
        start_x = float(self.x)
        start_y = float(self.y)

        #     # STAGED MOVEMENT SPEEDS
        # fast_speed = 0.03      # Full speed
        # slow_speed = 0.015     # Deceleration speed
        # crawl_speed = 0.005     # Final approach speed

        #         # DISTANCE ZONES
        # decel_zone = actual_target * 0.5   # Start slowing at 70% (15.4cm)
        # crawl_zone = actual_target * 0.8  # Crawl at 85% (18.7cm)


        # Move forward with timeout protection
        start_time = time.time()
        timeout = 15.0  # 10 second timeout
        
        while True:
            # Calculate distance traveled
            dx = self.x - start_x
            dy = self.y - start_y
            distance_traveled = float(math.hypot(dx, dy))
            
            # SPEED CONTROL
            # if distance_traveled < decel_zone:
            #     self.linear_vel = fast_speed
            #     self.get_logger().debug(f'🏃 Fast: {distance_traveled:.1f}cm')
            # elif distance_traveled < crawl_zone:
            #     self.linear_vel = slow_speed
            #     self.get_logger().debug(f'🚶 Slow: {distance_traveled:.1f}cm')
            # else:
            #     self.linear_vel = crawl_speed
            #     self.get_logger().debug(f'🐌 Crawl: {distance_traveled:.1f}cm')

            self.linear_vel = 0.02
            
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)  
            
            # STOP CONDITIONS
            if distance_traveled >= actual_target:
                self.get_logger().info(f'🛑 Stopping at: {distance_traveled:.1f}cm')
                break
                
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⏰ Movement timeout at {distance_traveled:.1f}cm')
                break
        
        
        self.linear_vel = 0.0
        self.publish_velocity()

        time.sleep(0.05)

            # MEASURE FINAL RESULT
        time.sleep(1)  # Let everything settle
        final_dx = self.x - start_x
        final_dy = self.y - start_y
        final_distance = math.hypot(final_dx, final_dy)
        
        error = abs(final_distance - distance_to_travel)
        self.get_logger().info(f'🎯 RESULT: Wanted {distance_to_travel:.1f}cm, got {final_distance:.1f}cm (error: {error:.1f}cm)')
          
            
       

    def obstacle_avoidance(self):
        self.get_logger().info("Making left turn to avoid obstacle")
        self.angular_vel = float(0.7)
        target_difference = 5 * math.pi / 12  # 90 degrees in radians 
        start_angle = self.normalize_angle(self.theta)
        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)
            current_angle = self.normalize_angle(self.theta)
            current_diff = current_angle - start_angle
            self.get_logger().info(f'🔄 Current angle difference: {math.degrees(current_diff):.1f}°')
            net_diff = target_difference - abs(current_diff)
            if ( abs(net_diff) < 0.15):
                self.get_logger().info(f'✅ Left turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

        self.angular_vel = float(0.0)  # Stop turning
        time.sleep(0.5)
        self.straight_movement()  # Continue moving forward after turn

        self.get_logger().info("Making right turn to avoid obstacle")
        self.angular_vel = float(-0.8)
        start_angle = self.normalize_angle(self.theta)

        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)
            current_angle = self.normalize_angle(self.theta)
            current_diff = start_angle - current_angle
            net_diff = target_difference - abs(current_diff)
            self.get_logger().info(f'🔄 Current angle difference: {math.degrees(current_diff):.1f}°')
            if (abs(net_diff) < 0.15):
                self.get_logger().info(f'✅ Right turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

        #Resetting angular velocity to 0 after avoidance
        self.angular_vel = float(0.0)
        time.sleep(0.5)  # Short pause to stabilize after avoidance 



            
    def left_turn(self,desired_heading):
        self.angular_vel = float(0.7)  # Moderate turn speed
        self.get_logger().info(f'🔄 Starting left turn to {math.degrees(desired_heading):.2f}')

        start_time = time.time()
        timeout = 10

        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update theta

            momentum_compensation = 0.3
            current_angle = self.normalize_angle(self.theta)
            angle_diff = self.normalize_angle(desired_heading - momentum_compensation - current_angle)

            self.get_logger().info(f'🔄 Current: {math.degrees(current_angle):.1f}°, Target: {math.degrees(desired_heading):.1f}°, Diff: {math.degrees(angle_diff):.1f}°')

             
            if abs(angle_diff) < 0.15:
                self.get_logger().info(f'✅ Left turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⏰ Left turn timeout after {timeout}s')
                break
        
        
        
    def right_turn(self,desired_heading):
        self.angular_vel = float(-0.7)  # Moderate turn speed
        self.get_logger().info(f'🔄 Starting right turn to {math.degrees(desired_heading):.1f}°')
        
        start_time = time.time()
        timeout = 10.0  # 10 second timeout
        
        while True:
            self.publish_velocity()  # 🐛 FIX: Added missing parentheses!
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update theta
            
            # Calculate current angle difference
            momentum_compensation = 0.3
            current_angle = self.normalize_angle(self.theta)
            angle_diff = self.normalize_angle(desired_heading - current_angle - momentum_compensation )
            
            self.get_logger().info(f'🔄 Current: {math.degrees(current_angle):.1f}°, Target: {math.degrees(desired_heading):.1f}°, Diff: {math.degrees(angle_diff):.1f}°')
            
            # Check if we've reached the target (within tolerance)
            if abs(angle_diff) < 0.15:
                self.get_logger().info(f'✅ Right turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break
                
            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⏰ Right turn timeout after {timeout}s')
                break
        
        self.angular_vel = float(0.0)  # Stop turning


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return float(angle)
        
        
    def publish_velocity(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist.linear.x = self.linear_vel
        twist_stamped.twist.angular.z = self.angular_vel
        
        self.vel_pub.publish(twist_stamped)
   
    def get_neighbors(self,current):
        neighbors = []
        x = current[0]
        y = current[1]
        
        if (0 <= x-1 <= self.height -1):
            neighbors.append([x-1,y])
        if (0 <= x+1 <= self.height -1):
            neighbors.append([x+1,y])
        if (0 <= y-1 <= self.width -1):
            neighbors.append([x,y-1])
        if (0 <= y+1 <= self.width -1):
            neighbors.append([x,y+1])
        
        self.get_logger().debug(f'🗺️ Neighbors of ({x},{y}): {neighbors}')
        return neighbors
            
    def is_free(self,neighbor):
        return (self.map_arr[neighbor[0]][neighbor[1]] != 100)
        
    def reconstruct_path(self,parent,goal):
        path = []
        starting_point = tuple(self.visited[0])
        
        current = tuple(goal)
        
        while(True):
            path.append(current)
            current = parent[current] 
            
            if(current == starting_point):
                path.append(current)
                break
            
        self.get_logger().info(f'🗺️ Reconstructed path: {path[::-1]}')
        return path[::-1]
        
def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()

    #Creating completion event
    completion_event = threading.Event()
    path_planner.completion_event = completion_event
    
    def spin_thread():
        rclpy.spin(path_planner)
        
    d_thread = threading.Thread(target=spin_thread)
    d_thread.daemon = True
    d_thread.start()
    
    try:
        while True:
            goal_grid_x = int(input("Please enter the goal grid coordinate(X) - grid_x :"))
            goal_grid_y = int(input("Please enter the goal grid coordinate(Y) - grid_y :"))
            
            path_planner.goal_grid_x = goal_grid_x
            path_planner.goal_grid_y = goal_grid_y

            completion_event.clear()
            
            result = path_planner.path_planner()
            path_planner.get_logger().info(f'🏁 Path planning result: {result}')

            if "executing" in result:
                print("Waiting for path execution to complete...")
                completion_event.wait(timeout=20)  # Wait for 20 seconds for path execution to complete
                if completion_event.is_set():
                    print("Path execution completed successfully!")
                else:
                    print("Path execution timed out or was not completed.")

        
    except Exception as e:
        path_planner.get_logger().error(f"An error occurred: {e}")
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()
      