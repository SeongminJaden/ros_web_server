import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time
import signal
import os

class AutoMappingNode(Node):
    def __init__(self):
        super().__init__('auto_mapping_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_callback)
        self.mapping_process = None
        self.start_time = None
        self.duration = 180  # 3 minutes
        self.last_change_time = None
        self.change_interval = 30  # Change direction every 30 seconds
        self.initial_position = None
        self.path = []

    def start_mapping(self):
        # Set up the ROS 2 environment
        os.system('source ~/cartoros2/install/setup.bash')
        
        # Start the mapping launch file
        self.mapping_process = subprocess.Popen(['ros2', 'launch', 'yahboom_bringup', 'Mapping_bring.launch.py'], env=os.environ)
        self.get_logger().info('Mapping started.')

    def stop_mapping(self):
        if self.mapping_process:
            self.mapping_process.send_signal(signal.SIGINT)
            self.mapping_process.wait()
            self.get_logger().info('Mapping process stopped.')

    def move_callback(self):
        if self.start_time is None:
            return  # Wait for the mapping to start

        current_time = time.time()
        if current_time - self.start_time < self.duration:
            twist = Twist()
            if current_time - self.last_change_time > self.change_interval:
                # Change direction every change_interval seconds
                self.last_change_time = current_time
                twist.linear.x = 0.5
                twist.angular.z = -0.3
            else:
                twist.linear.x = 0.5
                twist.angular.z = 0.3
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.save_map()
            self.return_to_start()
            self.stop_mapping()
            rclpy.shutdown()

    def save_map(self):
        self.get_logger().info('Saving the map...')
        # Finish trajectory
        subprocess.run(['ros2', 'service', 'call', '/finish_trajectory', 'cartographer_ros_msgs/srv/FinishTrajectory', '{trajectory_id: 0}'])
        # Write state
        subprocess.run(['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState', '{filename: "/home/yahboom/cartoros2/data/maps/mymap.pbstream"}'])
        # Convert to ROS map
        subprocess.run(['ros2', 'run', 'cartographer_ros', 'pbstream_to_ros_map_node', '-map_filestem=/home/yahboom/cartoros2/data/maps/mymap', '-pbstream_filename=/home/yahboom/cartoros2/data/maps/mymap.pbstream', '-resolution=0.05'])
        self.get_logger().info('Map saved successfully')

    def return_to_start(self):
        self.get_logger().info('Returning to start position')
        # In the absence of actual odom data, this example simply stops the robot.
        # You can implement a more sophisticated return mechanism using other sensors or pre-defined paths.

def main(args=None):
    rclpy.init(args=args)
    node = AutoMappingNode()
    
    # Start the mapping process before spinning the node
    node.start_mapping()
    node.get_logger().info('Waiting for 5 seconds to let mapping node initialize...')
    time.sleep(5)  # Wait for the mapping node to initialize
    
    node.start_time = time.time()
    node.last_change_time = node.start_time
    node.get_logger().info('Starting to move the robot for mapping...')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
