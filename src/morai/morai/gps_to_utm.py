import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import pyproj

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('utm_node')  # 고유한 노드 이름 부여
        self.publisher = self.create_publisher(PointStamped, '/Local/utm', 1)
        self.subscription = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 1)
        # 대한민국 경기도, 충청도 지역 UTM 52S 좌표계로 변환
        self.transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def gps_callback(self, msg):
        # NavSatFix 데이터를 읽어옴
        if msg.latitude and msg.longitude:
            lat = msg.latitude
            lon = msg.longitude

            # 위경도 좌표를 UTM 좌표로 변환
            easting, northing = self.transformer.transform(lon, lat)

            # UTM 좌표로 포인트 메시지 생성
            utm_point = PointStamped()
            utm_point.header.stamp = self.get_clock().now().to_msg()
            utm_point.header.frame_id = 'map'
            utm_point.point.x = easting
            utm_point.point.y = northing
            utm_point.point.z = msg.altitude

            self.publisher.publish(utm_point)
            # self.get_logger().info(f"Published UTM Point: {utm_point}\n")
            # self.get_logger().info(f"Received GPS data - latitude: {lat}, longitude: {lon}\n")

def main(args=None):
    rclpy.init(args=args)
    odom_publisher_node = OdomPublisher()
    rclpy.spin(odom_publisher_node)
    odom_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
