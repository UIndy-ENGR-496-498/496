import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class MS200LidarNode(Node):
    def __init__(self):
        super().__init__('ms200_lidar_node')
        
        # ✅ Explicitly set topic name to '/scan' (common convention in ROS 2)
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        
         # ✅ Increase timer frequency for more frequent publishing
        self.timer = self.create_timer(0.05, self.publish_scan_data)  # Publishes every 50ms
        
        self.get_logger().info('✅ MS200 Lidar Node Started')

    def publish_scan_data(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"

        # ✅ Proper Angle Definitions
        scan_msg.angle_min = -1.57  # Start angle (-90 degrees)
        scan_msg.angle_max = 1.57   # End angle (90 degrees)
        scan_msg.angle_increment = 0.01  # Small step for finer resolution

        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.05  # Should match timer (50ms)
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # ✅ Generate example Lidar scan data (pretend all points are at 0.5m)
        num_points = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        scan_msg.ranges = [0.5] * num_points

        # ✅ Publish the scan message
        self.publisher_.publish(scan_msg)
        self.get_logger().info(f'📡 Publishing Lidar Data ({num_points} points)')

def main(args=None):
    rclpy.init(args=args)
    node = MS200LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

