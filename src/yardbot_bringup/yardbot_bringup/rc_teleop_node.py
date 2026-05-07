import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import serial
import math


def gToMperS(g: float) -> float:
    """
    Convert gravitational acceleration (g) to meters per second squared (m/s^2).
    """
    return g * 9.80665

def degPerSecToRadPerSec(degPerSec: float) -> float:
    """
    Convert degrees per second to radians per second.
    """
    return degPerSec * math.pi / 180.0

class RCTeleopNode(Node):
    def __init__(self):
        super().__init__('rc_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.ser = serial.Serial('/dev/arduino', 115200, timeout=1)
        self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line.startswith('>'):
                    pass
                    # self.get_logger().info("Arduino debug message: " + line)
                else:
                    line = line[1:]  # Remove '>' prefix
                    self.parse_and_publish(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def parse_and_publish(self, line: str):
        # Expect: RC:linear=0.500,angular=-0.312,estop=0
        # or
        # IMU:ax=0.1,ay=0.2,az=0.3,gx=0.4,gy=0.5,gz=0.6,roll=0.7,pitch=0.8,yaw=0.9
            prefix, line = line.split(':', 1)
            match prefix:
                case 'RC':
                    self.parse_rc_message(line)
                case 'IMU':
                    self.parse_imu_message(line)
                case _:
                    self.get_logger().warn(f'Unknown message prefix: {prefix}')

    def parse_rc_message(self, line: str):
        try:
            if not line.startswith('RC:'):
                self.get_logger().warn(f'Invalid message format: {line}')
                return
            line = line[3:]  # Remove 'RC:' prefix
            parts = dict(p.split('=') for p in line.split(','))
            linear  = float(parts['linear'])
            angular = float(parts['angular'])
            estop   = parts['estop'].strip() == '1'
        except Exception as e:
            self.get_logger().warn(f'Error parsing RC message: {e}')

        estop_msg = Bool()
        estop_msg.data = estop
        self.estop_pub.publish(estop_msg)

        twist = Twist()
        if not estop:
            twist.linear.x  = linear  * 0.3   # scale to max m/s
            twist.angular.z = angular * 1.0    # scale to max rad/s
        self.cmd_pub.publish(twist)
        
    def parse_imu_message(self, line: str):
        try:
            if not line.startswith('IMU:'):
                self.get_logger().warn(f'Invalid message format: {line}')
                return
            line = line[4:]  # Remove 'IMU:' prefix
            parts = dict(p.split('=') for p in line.split(','))
            ax = gToMperS(float(parts['ax']))
            ay = gToMperS(float(parts['ay']))
            az = gToMperS(float(parts['az']))
            gx = degPerSecToRadPerSec(float(parts['gx']))
            gy = degPerSecToRadPerSec(float(parts['gy']))
            gz = degPerSecToRadPerSec(float(parts['gz']))
            roll = float(parts['roll'])
            pitch = float(parts['pitch'])
            yaw = float(parts['yaw'])
            # Publish the IMU data
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.orientation.x = roll
            imu_msg.orientation.y = pitch
            imu_msg.orientation.z = yaw
            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f'Error parsing IMU message: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RCTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()