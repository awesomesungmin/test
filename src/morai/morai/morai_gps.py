import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')  # 고유한 노드 이름 부여
        self.publisher_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('127.0.0.1', 2234)  # 설정에 맞게 변경
        self.sock.bind(self.server_address)
        self.sock.settimeout(1.0)  # Timeout 설정 (1초)
        self.timer_period = 0.5  # seconds (필요에 따라 변경)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(9236)

            if data:
                gps_data = data.decode('utf-8')

                if gps_data.startswith('$GPRMC'):
                    fields = gps_data.split(',')
                    lat_val = float(fields[3])
                    lon_val = float(fields[5])

                    # Convert from degrees minutes to decimal degrees
                    lat_deg = int(lat_val / 100)
                    lon_deg = int(lon_val / 100)

                    lat_min = lat_val - lat_deg * 100
                    lon_min = lon_val - lon_deg * 100

                    latitude = lat_deg + (lat_min / 60)
                    longitude = lon_deg + (lon_min / 60)

                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude

                    self.publisher_.publish(msg)
        except socket.timeout:
            pass  # Timeout 발생 시 아무 것도 하지 않음

def main(args=None):
    rclpy.init(args=args)

    gps_pub_node = GpsPublisher()

    rclpy.spin(gps_pub_node)

    gps_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
