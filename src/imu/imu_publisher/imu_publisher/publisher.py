import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial  # For serial communication with the IMU
import struct  # For parsing binary data from IMU

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.05, self.publish_imu_data)  # 20 Hz
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

    def publish_imu_data(self):
        try:
            # Read a line of data from the IMU (assuming ASCII data)
            line = self.serial_port.readline().decode('utf-8').strip()
            data = line.split(',')

            # Example IMU data format: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = map(float, data)

            # Create an Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Populate IMU message
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Publish the message
            self.publisher_.publish(imu_msg)
            self.get_logger().info(f'Published IMU data: {imu_msg}')

        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



