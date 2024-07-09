import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import struct
import math

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')  # 고유한 노드 이름 부여
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.publisher_heading = self.create_publisher(Float64, '/Local/heading', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('127.0.0.1', 1237)  # 설정에 맞게 변경
        self.sock.bind(self.server_address)
        self.sock.settimeout(1.0)  # Timeout 설정 (1초)
        self.timer_period = 0.5  # seconds (필요에 따라 변경)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.parsed_data = []

    def quaternion_to_euler(self, q0, q1, q2, q3):
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
        pitch = math.asin(2 * (q0 * q2 - q3 * q1))
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
        
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        
        return roll, pitch, yaw

    def timer_callback(self):
        try:
            raw_data, sender = self.sock.recvfrom(1236)
            header = raw_data[0:9].decode()

            if header == '#IMUData$':
                data_length = struct.unpack('i', raw_data[9:13])
                imu_data = struct.unpack('10d', raw_data[25:105])
                self.parsed_data = imu_data

                msg = Imu()
                msg_heading = Float64()

                msg.orientation.w = round(self.parsed_data[1], 2)
                msg.orientation.x = round(self.parsed_data[2], 2)
                msg.orientation.y = round(self.parsed_data[3], 2)
                msg.orientation.z = round(self.parsed_data[4], 2)

                roll, pitch, yaw = self.quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
                msg_heading.data = yaw / 180 * math.pi
                print("heading : ", msg_heading.data)
                self.publisher_heading.publish(msg_heading)
                self.publisher_.publish(msg)
        except socket.timeout:
            pass  # Timeout 발생 시 아무 것도 하지 않음

def main(args=None):
    rclpy.init(args=args)

    imu_pub_node = ImuPublisher()

    rclpy.spin(imu_pub_node)

    imu_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
