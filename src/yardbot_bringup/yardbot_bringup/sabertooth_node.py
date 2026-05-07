#!/usr/bin/env python3
"""
sabertooth_test.py
Minimal ROS2 node for Sabertooth 2x32 packetized serial testing.
Subscribes to /cmd_vel and drives motors 1 & 2 accordingly.
"""

import threading
import time
from typing import Literal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial

ADDRESS = 128                  # default Sabertooth address


def calculate_crc7(data: bytes) -> int:
    crc = 0x7F
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x76
            else:
                crc >>= 1
    return crc ^ 0x7F


def calculate_crc14(data: bytes) -> int:
    crc = 0x3FFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x22F0
            else:
                crc >>= 1
    return crc ^ 0x3FFF

def verify_crc7(packet: bytes):
    """
    Checks if the last byte of the packet matches 
    the CRC-7 of the preceding bytes.
    """
    if len(packet) < 2:
        return False
        
    received_crc = packet[-1]
    calculated_crc = calculate_crc7(packet[:-1])
    return received_crc == calculated_crc

def get_14bit_value(low_byte: int, high_byte: int) -> int:
    # Reconstruct the 14-bit value
    # Shift the high byte left by 7 and OR it with the low byte
    return (high_byte << 7) | low_byte

def sabertooth_checksum(address, command, value):
    return (address + command + value) & 0x7F

class SabertoothNode(Node):
    def __init__(self):
        super().__init__('sabertooth_node')
        baud_rate_param = self.declare_parameter('baud_rate', 9600)
        serial_port_param = self.declare_parameter('serial_port', '/dev/sabertooth')
        baud_rate = baud_rate_param.get_parameter_value().integer_value
        serial_port = serial_port_param.get_parameter_value().string_value
        self.max_linear_velocity = self.declare_parameter('max_linear_velocity', 0.2).get_parameter_value().double_value
        self.max_angular_velocity = self.declare_parameter('max_angular_velocity', 1.0).get_parameter_value().double_value
        self.ser = serial.Serial(serial_port, baud_rate, timeout=3)
        self.get_logger().info(f'Opened serial port {serial_port} at {baud_rate} baud')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.temp_publisher = self.create_publisher(Float32MultiArray, 'motor_temps', 10)
        self.temp_poll_timer = self.create_timer(1.0, self.timer_callback) # Poll every 1s
        self._serial_lock = threading.Lock()
        self.motor_command_pending = False

    def send_motor(self, command, value):
        """Send a single packetized serial command."""
        value = max(0, min(127, value))
        checksum = sabertooth_checksum(ADDRESS, command, value)
        packet = bytes([ADDRESS, command, value, checksum])
        self.get_logger().info(f'Sending: {list(packet)}')
        # Set a flag to indicate a motor command is pending, so we can avoid serial collisions when reading temps
        self.motor_command_pending = True
        with self._serial_lock:
            self.ser.write(packet)
        self.motor_command_pending = False

    # Scale to Sabertooth 0-127 range
    def to_sabertooth(self, v: float) -> int:
        return int(abs(v) / self.max_linear_velocity * 127)

    def cmd_vel_cb(self, msg: Twist):
        linear = max(-self.max_linear_velocity, min(self.max_linear_velocity, msg.linear.x))
        angular = msg.angular.z  # rad/s, turn

        # Simple differential drive mixing
        left = linear - angular * 0.5
        right = linear + angular * 0.5


        # Packetized serial commands:
        # Command 0 = M1 forward, 1 = M1 backward
        # Command 4 = M2 forward, 5 = M2 backward
        if left >= 0:
            self.send_motor(0, self.to_sabertooth(left))
        else:
            self.send_motor(1, self.to_sabertooth(left))

        if right >= 0:
            self.send_motor(4, self.to_sabertooth(right))
        else:
            self.send_motor(5, self.to_sabertooth(right))

    def destroy_node(self):
        self.send_motor(0, 0)  # stop M1
        self.send_motor(4, 0)  # stop M2
        self.ser.close()
        super().destroy_node()

    def get_temp(self, motor_num: Literal[1, 2]):
        # Packet: [Addr + 112 (to signify CRC), Cmd, Value, Data1, Data2, CRC]
        header = bytes([ADDRESS + 112, 41, 64])
        header_crc = calculate_crc7(header)
        data = bytes([ord('M'), ord(str(motor_num))])  # ASCII 'M' followed by ASCII motor number
        data_crc = calculate_crc14(data)
        data_crc_low  = (data_crc >> 0) & 0x7F
        data_crc_high = (data_crc >> 7) & 0x7F
        packet = bytes([*header, header_crc, *data, data_crc_low, data_crc_high])
        self.get_logger().info(f'Full packet: {list(packet)}')


        if self.motor_command_pending:
            self.get_logger().warn("Motor command pending, skipping temp read to avoid serial collision")
            return None
        
        with self._serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            response = self.ser.read(10)
        
        self.get_logger().info(f"Response: {response}")
        
        if len(response) != 10:
            self.get_logger().error("Invalid response length")
            return None
        if response[0] != ADDRESS + 112:
            self.get_logger().error("Invalid address")
            return None
        if response[1] != 73:
            self.get_logger().error("Invalid reply ID")
            return None
        if calculate_crc7(response[:3]) != response[3]:
            self.get_logger().error("Invalid header CRC")
            return None
        if calculate_crc14(response[4:8]) != get_14bit_value(response[8], response[9]):
            self.get_logger().error("Invalid data CRC")
            return None
        raw_value = get_14bit_value(response[4], response[5])

        # degrees in Celsius
        return float(raw_value)

    def timer_callback(self):
        t1 = self.get_temp(1) # M1 Temperature
        t2 = self.get_temp(2) # M2 Temperature
        
        if t1 is not None and t2 is not None:
            msg = Float32MultiArray()
            msg.data = [t1, t2]
            self.temp_publisher.publish(msg)
            self.get_logger().info(f'Temps: M1={t1}C, M2={t2}C')


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