import rclpy
from rclpy.node import Node
from roomba_interfaces.msg import LocalizationData
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped
import numpy as np
import threading
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.get_logger().info('🤖 Path Planner Node Started')
        
        self.loc_subscriber = self.create_subscription(LocalizationData,"/loc",self.loc_callback,10)
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.vel_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel',10)

        self.linear_vel = 0.0
        self.angular_vel = 0.0
    
        self.grid_x = 0
        self.grid_y = 0
        
        self.goal_grid_x = 0
        self.goal_grid_y = 0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.cell_side = 22
        
        self.confidence = 0.0
        
        self.map_arr = None
        
        self.height = 0
        self.width = 0
        
        self.queue = []
        self.visited = []
        self.parent = {}
        self.goal = None
        
        self.paths = []    # Has only one path as of now
        self.optimized_path = None

        self.get_logger().info('✅ Path Planner initialized and ready')
        
    def loc_callback(self,msg):
        self.grid_x,self.grid_y,self.theta = msg.grid_x,msg.grid_y,msg.theta
        self.x,self.y = msg.x,msg.y
        self.confidence = msg.confidence
        
    def map_callback(self,msg):
        self.height = msg.info.height
        self.width = msg.info.width
        self.map_arr = np.array(msg.data).reshape((self.height,self.width))

                # Log map statistics
        free_cells = np.count_nonzero(self.map_arr == -1)
        obstacle_cells = np.count_nonzero(self.map_arr == 100)
        unknown_cells = np.count_nonzero(self.map_arr == 0)
        self.get_logger().info(f'📊 Map stats: {free_cells} free, {obstacle_cells} obstacles, {unknown_cells} unknown')
        
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

        self.queue.append([self.grid_x,self.grid_y])
        self.visited.append([self.grid_x,self.grid_y])
        self.goal = [self.goal_grid_x,self.goal_grid_y]

        nodes_explored = 0


        
        while self.queue :
            current = self.queue.pop(0)
            nodes_explored += 1
            
            if(current == self.goal ):
                self.get_logger().info(f'✅ Path found! Explored {nodes_explored} nodes')
                self.paths.append(self.reconstruct_path(self.parent,self.goal))
                self.optimized_path = self.paths[0]  # As of now, just taking as it is since there's only one path
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
        if (self.confidence > 50):
            self.get_logger().info(f'🚀 Starting path execution with confidence {self.confidence:.1f}%')
            
            #Step - 1 ( Dealing with theta difference)
            
            sub_goal_grid_x = self.optimized_path[1][0]
            sub_goal_grid_y = self.optimized_path[1][1]
            
            sub_goal_y = (sub_goal_grid_x - self.grid_x)*self.cell_side + self.y
            sub_goal_x = (sub_goal_grid_y - self.grid_y)*self.cell_side + self.x
            
            desired_heading = math.atan2((sub_goal_grid_x - self.grid_x)*self.cell_side,(sub_goal_grid_y - self.grid_y)*self.cell_side)
            
            difference = desired_heading - self.theta
            self.get_logger().info(f'🧭 Current heading: {math.degrees(self.theta):.1f}°, Target: {math.degrees(desired_heading):.1f}°, Difference: {math.degrees(difference):.1f}°')
            # Skipping the no turn, and very minute difference for now
            if abs(difference) > 0.1:
                if(difference > 0):
                    self.left_turn(desired_heading)
                else:
                    self.right_turn(desired_heading)
            else:
                self.get_logger().info('🛑 No significant turn needed, proceeding straight')
                
            self.angular_vel = 0 #For a good practise, setting the angular velocity back to 0
            self.get_logger().info('✅ Turn complete')
                
            #Step - 2 ( Dealing with straight movement)
            a1 = (self.goal_grid_y - self.grid_y)
            b1 = (self.goal_grid_x - self.grid_y)
            distance_to_travel = math.hypot(a1,b1)*self.cell_side

            self.get_logger().info(f'📏 Moving forward {distance_to_travel:.1f}cm')
            
            curr_x = self.x
            curr_y = self.y
            d = 0
            self.linear_vel = 0.1
            
            while(d <= distance_to_travel):
                self.publish_velocity()
                rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks to update position

                a2 = (curr_x - self.x)
                b2 = (curr_y - self.y)
                d = math.hypot(a2,b2)

                if int(d) % 10 == 0 and int(d) > 0:
                    self.get_logger().debug(f'📍 Progress: {d:.1f}cm / {distance_to_travel:.1f}cm')
                
            self.linear_vel = 0  # Setting the linear_vel back to 0 after the required movement
            self.get_logger().info('✅ Path execution completed!')
            
        else:
            self.get_logger().warn(f'⚠️  Low confidence ({self.confidence:.1f}%) - stopping for safety')

        if hasattr(self, 'completion_event'):
            self.completion_event.set() #Signalling completion


            
    def left_turn(self,desired_heading):
        self.angular_vel = 0.5
        self.get_logger().info(f'🔄 Starting left turn to {desired_heading:.2f}')
        while(self.theta < desired_heading):
            self.publish_velocity()

            # Process callbacks to update theta
            rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().debug(f'Current: {self.theta:.2f}, Target: {desired_heading:.2f}')
        
        
        
    def right_turn(self,desired_heading):
        self.angular_vel = -0.5
        self.get_logger().info(f'🔄 Starting left turn to {desired_heading:.2f}')
        while(self.theta > desired_heading):
            self.publish_velocity

             # Process callbacks to update theta
            rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().debug(f'Current: {self.theta:.2f}, Target: {desired_heading:.2f}')
        
        
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
            
            result = path_planner.path_planner()
            path_planner.get_logger().info(f'🏁 Path planning result: {result}')

            completion_event.wait()
            print(" Path execution completed!!")
        
    except Exception as e:
        path_planner.get_logger().error(f"An error occurred: {e}")
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()
      