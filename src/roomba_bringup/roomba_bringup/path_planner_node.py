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

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.get_logger().info('🤖 Path Planner Node Started')

        self.sensor_sub = self.create_subscription(SensorData, '/wheel_states', self.sensor_callback, 10)
        
        self.loc_subscriber = self.create_subscription(LocalizationData,"/loc",self.loc_callback,10)
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.vel_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel',10)

        self.linear_vel = float(0.0)
        self.angular_vel = float(0.0)
    
        self.grid_x = int(0)
        self.grid_y = int(0)
        
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
        
    def map_callback(self,msg):
        self.height = int(msg.info.height)
        self.width = int(msg.info.width)
        self.map_arr = np.array(msg.data).reshape((self.height,self.width))
        
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
                self.execute_path()

                self.get_logger().info(f'📏 Path length: {len(self.optimized_path)} waypoints')
                self.get_logger().debug(f'🛤️  Path: {self.optimized_path}')
                
                return "Path found and executing"
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in self.visited and self.is_free(neighbor):
                    self.visited.append(neighbor)
                    self.parent[tuple(neighbor)] = tuple(current)
                    self.queue.append(neighbor)
                    
        self.get_logger().error(f'❌ No path found after exploring {nodes_explored} nodes')
        return "No Path Found"
        
    def execute_path(self):
        if (self.confidence > 10):
            self.get_logger().info(f'🚀 Starting path execution with confidence {self.confidence:.1f}%')
            
            #Step - 1 ( Dealing with theta difference)

            for i in range(1, len(self.optimized_path)):
            
                next_grid_x = self.optimized_path[i][0]
                next_grid_y = self.optimized_path[i][1] 
                
                # sub_goal_y = (next_grid_x - self.grid_x)*self.cell_side + self.y
                # sub_goal_x = (next_grid_y - self.grid_y)*self.cell_side + self.x
                
                desired_heading = float(math.atan2((next_grid_x - self.grid_x),(next_grid_y - self.grid_y)))           # Always gonna be 90 degrees

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
                self.publish_velocity()
                self.get_logger().info('✅ Turn complete')

                rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update obstacle distance

                #Dealing with straight movement
                if (self.obstacle_dist < 2*self.cell_side):
                    self.get_logger().warn(f'⚠️  Obstacle detected at {self.obstacle_dist:.1f}cm - stopping for safety')
                    self.obstacle_avoidance()  # Handle obstacle avoidance
                    self.path_planner()  # Restart path planning
                    return
                else:
                    self.get_logger().info(f'✅ No obstacles detected, proceeding to next waypoint ({next_grid_x},{next_grid_y})')  
                    self.straight_movement()
                    
                
            
        else:
            self.get_logger().warn(f'⚠️  Low confidence ({self.confidence:.1f}%) - stopping for safety')

        if hasattr(self, 'completion_event'):
            self.completion_event.set() #Signalling completion


    def straight_movement(self):
        # Step - 2 (Dealing with straight movement)
        distance_to_travel = float(self.cell_side)

        self.get_logger().info(f'📏 Moving forward {distance_to_travel:.1f}cm')
        
        start_x = float(self.x)
        start_y = float(self.y)

        self.linear_vel = float(0.1)

        # Move forward with timeout protection
        start_time = time.time()
        timeout = 10.0  # 10 second timeout
        
        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update position
            
            # Calculate distance traveled
            dx = self.x - start_x
            dy = self.y - start_y
            distance_traveled = float(math.hypot(dx, dy))
            
            # Check completion conditions
            if distance_traveled >= distance_to_travel:
                self.get_logger().info(f'📍 Target distance reached: {distance_traveled:.1f}cm')
                break
                
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⏰ Movement timeout after {timeout}s')
                break
            
            # Progress logging
            if int(distance_traveled) % 5 == 0 and int(distance_traveled) > 0:
                self.get_logger().debug(f'📍 Progress: {distance_traveled:.1f}cm / {distance_to_travel:.1f}cm')
            
        self.linear_vel = float(0.0)  # Stop moving
        self.publish_velocity()  # Send stop command
        self.get_logger().info('✅ Path execution completed!')

    def obstacle_avoidance(self):
        self.get_logger().info("Making left turn to avoid obstacle")
        self.angular_vel = float(0.2)
        target_difference = math.pi / 2  # 90 degrees in radians 
        start_angle = self.normalize_angle(self.theta)
        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)
            current_angle = self.normalize_angle(self.theta)
            if ( current_angle - start_angle > target_difference):
                self.get_logger().info(f'✅ Left turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

        self.straight_movement()  # Continue moving forward after turn

        self.get_logger().info("Making right turn to avoid obstacle")
        self.angular_vel = float(-0.2)
        start_angle = self.normalize_angle(self.theta)

        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)
            current_angle = self.normalize_angle(self.theta)
            if ( start_angle - current_angle > target_difference    ):
                self.get_logger().info(f'✅ Right turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

        #Resetting angular velocity to 0 after avoidance
        self.angular_vel = float(0.0)
        self.publish_velocity()  # Send stop command



            
    def left_turn(self,desired_heading):
        self.angular_vel = float(0.2)  # Moderate turn speed
        self.get_logger().info(f'🔄 Starting left turn to {math.degrees(desired_heading):.2f}')

        start_time = time.time()
        timeout = 10

        while True:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update theta

            current_angle = self.normalize_angle(self.theta)
            angle_diff = self.normalize_angle(desired_heading - current_angle)

            self.get_logger().info(f'🔄 Current: {math.degrees(current_angle):.1f}°, Target: {math.degrees(desired_heading):.1f}°, Diff: {math.degrees(angle_diff):.1f}°')

             
            if abs(angle_diff) < 0.15:
                self.get_logger().info(f'✅ Left turn completed! Final angle: {math.degrees(current_angle):.1f}°')
                break

            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⏰ Left turn timeout after {timeout}s')
                break
        
        
        
    def right_turn(self,desired_heading):
        self.angular_vel = float(-0.2)  # Moderate turn speed
        self.get_logger().info(f'🔄 Starting right turn to {math.degrees(desired_heading):.1f}°')
        
        start_time = time.time()
        timeout = 10.0  # 10 second timeout
        
        while True:
            self.publish_velocity()  # 🐛 FIX: Added missing parentheses!
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update theta
            
            # Calculate current angle difference
            current_angle = self.normalize_angle(self.theta)
            angle_diff = self.normalize_angle(desired_heading - current_angle)
            
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
                completion_event.wait(timeout=20)  # Wait for 30 seconds for path execution to complete
                if completion_event.is_set():
                    print("Path execution completed successfully!")
                else:
                    print("Path execution timed out or was not completed.")

        
    except Exception as e:
        path_planner.get_logger().error(f"An error occurred: {e}")
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()
      