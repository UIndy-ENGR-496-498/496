#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math

from sensor_msgs.msg import Imu
import tf_transformations


class ArduinoImuNode(Node):
    def __init__(self):
        super().__init__('arduino_imu_node')

        # Adjust to match your actual serial port and baud
        port = '/dev/ttyIMU'  # or '/dev/ttyUSB0', etc.
        baud = 19200

        # Try opening the serial port
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"Connected to {port} at {baud} baud.")
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise SystemExit(1)

        # Publisher for Imu messages on the /imu/data topic
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Read from the serial port ~50 times per second
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        """
        Periodically read a line from the Arduino, parse it, and publish an Imu message.
        """
        line_bytes = self.ser.readline()
        line = line_bytes.decode('utf-8', errors='ignore').strip()

        self.get_logger().info(f"Raw data received: {line}")

        if not line or "AccError" in line or "GyroError" in line:
            return

        parts = line.split('/')
        if len(parts) != 6:
            self.get_logger().warn(f"Unexpected IMU data format: {line}")
            return

        try:
            roll_deg, pitch_deg, yaw_deg = map(float, parts[0:3])
            acc_x, acc_y, acc_z = map(float, parts[3:6])
        except ValueError:
            self.get_logger().warn(f"Could not convert to float: {line}")
            return

        # Convert degrees to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        # Convert RPY -> quaternion
        q = tf_transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

        imu_msg = Imu()
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # Populate linear acceleration
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z

        # Publish the message
        self.imu_pub.publish(imu_msg)

        self.get_logger().info(f"Published IMU orientation and acceleration.")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

