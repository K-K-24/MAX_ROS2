import rclpy
from rclpy.node import Node
from roomba_interfaces.msg import SensorData
from roomba_interfaces.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import json
import os
from datetime import datetime

class SimpleMapperNode(Node):

	def __init__(self):
		super().__init__('simple_mapper')

		self.sensor_sub = self.create_subscription(SensorData, '/wheel_states', self.sensor_callback, 10)

		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

		self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

		self.timer = self.create_timer(1, self.mapper_callback)
		
		# Add auto-save timer (saves map every 30 seconds)
		self.auto_save_timer = self.create_timer(30, self.auto_save_map)

		self.map_height = 1
		self.map_width = 1
		self.cell_side = 22   # Length of the robot in cm

		self.height_cells = 4
		self.width_cells = 4

		self.map_array = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

		#Starting robot in the center of the map
		self.grid_x = self.height_cells // 2 - 1    
		self.grid_y = self.width_cells // 2 - 1

		self.sum_grid_x = self.grid_x
		self.sum_grid_y = self.grid_y

		self.obstacle_dist = 0.0

		self.odom_count = 0
		self.sensor_count = 0
		self.mapper_count = 0

		self.last_x = 0.0       # Previous x coordinate of robot
		self.last_y = 0.0       # Previous y coordinate of robot

		self.next_cell_x = 0    # Coordinates of the next grid cell in front of the robot
		self.next_cell_y = 0

		# Map save configuration
		self.map_save_directory = os.path.expanduser('~/ros2_ws/maps/')
		self.ensure_map_directory()

		self.get_logger().info('🗺️  Simple Mapper Node Started!')
		self.get_logger().info(f'📐 Map setup: {self.height_cells}x{self.width_cells} cells, cell_size={self.cell_side}cm')
		self.get_logger().info(f'🎯 Robot starts at grid center: ({self.grid_x}, {self.grid_y})')
		self.get_logger().info(f'💾 Maps will be saved to: {self.map_save_directory}')
		self.get_logger().info(f'⌨️  Press Ctrl+S (if supported) or wait 30s for auto-save')

	def ensure_map_directory(self):
		"""Create maps directory if it doesn't exist"""
		try:
			os.makedirs(self.map_save_directory, exist_ok=True)
			self.get_logger().info(f'📁 Map directory ready: {self.map_save_directory}')
		except Exception as e:
			self.get_logger().error(f'❌ Failed to create map directory: {e}')
			self.map_save_directory = './'  # Fallback to current directory

	def save_map_to_file(self, filename=None):
		"""Save current map to JSON file"""
		try:
			if filename is None:
				timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
				filename = f"roomba_map_{timestamp}.json"
			
			filepath = os.path.join(self.map_save_directory, filename)
			
			# Prepare map data
			map_data = {
				'metadata': {
					'created_at': datetime.now().isoformat(),
					'height_cells': self.height_cells,
					'width_cells': self.width_cells,
					'cell_size_cm': self.cell_side,
					'map_height_m': self.map_height,
					'map_width_m': self.map_width,
					'starting_grid_x': self.height_cells // 2-1,
					'starting_grid_y': self.width_cells // 2-1
				},
				'map_array': self.map_array.tolist(),
				'statistics': {
					'free_cells': int(np.count_nonzero(self.map_array == -1)),
					'obstacle_cells': int(np.count_nonzero(self.map_array == 100)),
					'unknown_cells': int(np.count_nonzero(self.map_array == 0)),
					'total_updates': self.mapper_count
				}
			}
			
			# Save to file
			with open(filepath, 'w') as f:
				json.dump(map_data, f, indent=2)
			
			# Also save latest map for easy loading
			latest_filepath = os.path.join(self.map_save_directory, 'latest_map.json')
			with open(latest_filepath, 'w') as f:
				json.dump(map_data, f, indent=2)
			
			self.get_logger().info(f'💾 Map saved successfully!')
			self.get_logger().info(f'📄 File: {filename}')
			self.get_logger().info(f'📊 Stats: {map_data["statistics"]["free_cells"]} free, {map_data["statistics"]["obstacle_cells"]} obstacles')
			
			return filepath
			
		except Exception as e:
			self.get_logger().error(f'❌ Failed to save map: {e}')
			return None

	def auto_save_map(self):
		"""Automatically save map every 30 seconds"""
		if self.mapper_count > 0:  # Only save if we have some mapping data
			self.save_map_to_file()

	def odom_callback(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta
		self.odom_count += 1

		if self.odom_count % 10 == 0:  # Print every 10th odometry message
			self.get_logger().info(f'📍 Odometry #{self.odom_count}: x={self.x:.2f}cm, y={self.y:.2f}cm, θ={math.degrees(self.theta):.1f}°')

	def sensor_callback(self, msg):
		self.obstacle_dist = msg.ultrasonic_distance

		self.sensor_count += 1
		if self.sensor_count % 5 == 0:  # Print every 5th sensor message
			self.get_logger().info(f'📡 Sensor #{self.sensor_count}: ultrasonic_distance={self.obstacle_dist:.2f}cm')

	def is_valid_cell(self, x, y):
		# Check if the cell coordinates are within the map bounds
		return 0 <= x < self.height_cells and 0 <= y < self.width_cells

	def publish_map(self):
		map_msg = OccupancyGrid()

		#Header
		map_msg.header.stamp = self.get_clock().now().to_msg()
		map_msg.header.frame_id = 'map'

		#Map metadata
		map_msg.info.resolution = self.cell_side / 100.0  # Convert cm to m
		map_msg.info.width = self.width_cells
		map_msg.info.height = self.height_cells

		#Map origin
		map_msg.info.origin.position.x = -0.88  # Centering the map origin
		map_msg.info.origin.position.y = -1.54
		map_msg.info.origin.position.z = 0.0
		map_msg.info.origin.orientation.w = 1.0  # No rotation

		map_msg.data = self.map_array.flatten().tolist()  # Flatten the 2D array to 1D list
		self.map_pub.publish(map_msg)
		self.get_logger().debug('🗺️  Published updated map!')

	def mapper_callback(self):
		
		self.mapper_count += 1
		
		self.get_logger().info(f'\n🔄 ===== MAPPER UPDATE #{self.mapper_count} =====')
		
		diff_x = (self.x - self.last_x) / self.cell_side     # Find the difference in terms of number of cells
		diff_y = (self.y - self.last_y) / self.cell_side

		self.get_logger().info(f'📏 Movement: diff_x={diff_x:.3f} cells, diff_y={diff_y:.3f} cells')

		old_grid_x = self.grid_x
		old_grid_y = self.grid_y

		self.sum_grid_x += diff_x
		self.sum_grid_y += diff_y

		self.grid_x = round(self.sum_grid_x)       
		self.grid_y = round(self.sum_grid_y)

		self.get_logger().info(f'🎯 Grid position: ({old_grid_x}, {old_grid_y}) → ({self.grid_x}, {self.grid_y})')

		not_moving = self.last_x == self.x and self.last_y == self.y
		self.get_logger().info(f'🚶 Movement status: {"STATIONARY" if not_moving else "MOVING"}')

		if not not_moving:
			next_x_raw = self.grid_x + round(math.cos(self.theta))   # Always finding the coord of the next cell with bounds checking
			next_y_raw = self.grid_y + round(math.sin(self.theta))		# dsin and dcos for the distance - d( Cell size)

			self.get_logger().info(f'👀 Next cell calculation: ({self.grid_x}, {self.grid_y}) + ({round(math.cos(self.theta))}, {round(math.sin(self.theta))}) = ({next_x_raw}, {next_y_raw})')

			#Only update map if next cell is valid
			if self.is_valid_cell(next_x_raw, next_y_raw):
				self.next_cell_x = next_x_raw
				self.next_cell_y = next_y_raw
			
				if self.obstacle_dist > self.cell_side:
					self.map_array[self.next_cell_x, self.next_cell_y] = -1
					self.get_logger().info(f'✅ Updated map cell ({self.next_cell_x}, {self.next_cell_y}) as Free space')
				else:
					self.map_array[self.next_cell_x, self.next_cell_y] = 100
					self.get_logger().info(f'🚧 Updated map cell ({self.next_cell_x}, {self.next_cell_y}) as Obstacle')

			else:	
				self.get_logger().warning(f'⚠️  Next cell ({next_x_raw}, {next_y_raw}) is out of bounds!')

		self.last_x = self.x        # Update the last coordinates after 1 second
		self.last_y = self.y
		
		#Marking robot position if it's valid
		if self.is_valid_cell(self.grid_x, self.grid_y):
			self.map_array[self.grid_x, self.grid_y] = -1
			self.get_logger().info(f'🤖 Marked robot position ({self.grid_x}, {self.grid_y}) as FREE')
		else:
			self.get_logger().warning(f'⚠️  Robot position ({self.grid_x}, {self.grid_y}) is out of bounds!')
	
		#Map statistics
		free_cells = np.count_nonzero(self.map_array == -1)
		obstacle_cells = np.count_nonzero(self.map_array == 100)
		unknown_cells = np.count_nonzero(self.map_array == 0)
		
		self.get_logger().info(f'📊 Map stats: {free_cells} free, {obstacle_cells} obstacles, {unknown_cells} unknown')
		self.get_logger().info('🔄 ===== END MAPPER UPDATE =====\n')

		self.publish_map()

	def destroy_node(self):
		"""Save map before shutting down"""
		self.get_logger().info('🛑 Shutting down mapper - saving final map...')
		self.save_map_to_file('final_map.json')
		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	simple_mapper = SimpleMapperNode()
	
	try:
		rclpy.spin(simple_mapper)
	except KeyboardInterrupt:
		simple_mapper.get_logger().info('🛑 Keyboard interrupt received')
	finally:
		simple_mapper.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()