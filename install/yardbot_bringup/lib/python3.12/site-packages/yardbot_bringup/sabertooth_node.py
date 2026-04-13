#!/usr/bin/env python3
"""
sabertooth_test.py
Minimal ROS2 node for Sabertooth 2x32 packetized serial testing.
Subscribes to /cmd_vel and drives motors 1 & 2 accordingly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

SERIAL_PORT = '/dev/sabertooth'  # adjust as needed
BAUD_RATE = 9600               # must match Sabertooth DIP switch config
ADDRESS = 128                  # default Sabertooth address

def sabertooth_checksum(address, command, value):
    return (address + command + value) & 0x7F

class SabertoothNode(Node):
    def __init__(self):
        super().__init__('sabertooth_test')
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.get_logger().info(f'Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.get_logger().info(f'Port open: {self.ser.is_open}')

    def send_motor(self, command, value):
        """Send a single packetized serial command."""
        value = max(0, min(127, value))
        checksum = sabertooth_checksum(ADDRESS, command, value)
        packet = bytes([ADDRESS, command, value, checksum])
        self.get_logger().info(f'Sending: {list(packet)}') 
        self.ser.write(packet)

    def cmd_vel_cb(self, msg: Twist):
        linear = msg.linear.x    # m/s, forward/back
        angular = msg.angular.z  # rad/s, turn

        # Simple differential drive mixing
        left = linear - angular * 0.5
        right = linear + angular * 0.5

        # Scale to Sabertooth 0-127 range
        def to_sabertooth(v):
            return int(abs(v) / 1.0 * 127)  # assumes max 1.0 m/s

        # Packetized serial commands:
        # Command 0 = M1 forward, 1 = M1 backward
        # Command 4 = M2 forward, 5 = M2 backward
        if left >= 0:
            self.send_motor(0, to_sabertooth(left))
        else:
            self.send_motor(1, to_sabertooth(left))

        if right >= 0:
            self.send_motor(4, to_sabertooth(right))
        else:
            self.send_motor(5, to_sabertooth(right))

    def destroy_node(self):
        self.send_motor(0, 0)  # stop M1
        self.send_motor(4, 0)  # stop M2
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SabertoothNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()