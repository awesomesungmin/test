import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import socket
import time
from threading import Thread, Event
import cv2
import numpy as np
import struct
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CAMConnector(Node):
    def __init__(self, network_type, destination_port):
        super().__init__('morai_camera_node')  # 고유한 노드 이름 부여
        self.camClient = None    
        self.networkType = network_type
        self.destination_port = destination_port
        self.connChk = False
        self.recvChk = False
        self.event = Event()
        self.TotalIMG = None

        self.publisher_ = self.create_publisher(Image, '/flir_camera/image_raw', 10)
        self.bridge = CvBridge()

    def connect(self, host, port, destination_port):
        if self.networkType == 'UDP':
            try:
                self.camClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.camClient.setblocking(False)
                self.camClient.settimeout(2)
                self.camClient.bind((host, port))       

                self.check_max_len()
                self.camRecvThread = Thread(target=self.loop, args=())
                self.camRecvThread.daemon = True 
                self.camRecvThread.start()
                self.connChk = True
                self.get_logger().info('Connected to camera')

            except Exception as e:
                self.get_logger().error(f'cam_connect : {e}')

    def disconnect(self):    
        if self.networkType == 'UDP':
            if self.connChk:
                self.connChk = False
                if self.camRecvThread.is_alive():
                    self.event.set()
                    self.camRecvThread.join()        
                self.camClient.close()
                self.get_logger().info('Disconnected from camera')

    def check_max_len(self):
        idx_list = b''
        r_step = 0
        while r_step < int(10):            
            UnitBlock, sender = self.camClient.recvfrom(65000)
            idx_list += UnitBlock[3:7]
            r_step += 1
        self.get_logger().info('Checked maximum length')

    def loop(self):
        while rclpy.ok():
            self.image()      
            if self.event.is_set():
                break

    def image(self):
        TotalBuffer = b''
        while True:
            try:
                UnitBlock, sender = self.camClient.recvfrom(65000)
                UnitIdx = struct.unpack('i', UnitBlock[11:15])[0]
                UnitSize = struct.unpack('i', UnitBlock[15:19])[0]
                Unitdata = UnitBlock[19:-2]
                UnitTail = UnitBlock[-2:]

                if UnitTail == b'EI':
                    TotalBuffer += Unitdata

                    self.TotalIMG = cv2.imdecode(np.frombuffer(TotalBuffer, np.uint8), 1)
                    self.img_byte = np.array(cv2.imencode('.jpg', self.TotalIMG)[1]).tobytes()
                    
                    TotalBuffer = b''
                    self.recvChk = True
                    self.get_logger().info('Received complete image')

                    # 이미지 퍼블리시
                    ros_img = self.getImg()
                    if ros_img is not None:
                        self.publisher_.publish(ros_img)
                        self.get_logger().info('Published image')

                    break

                else:
                    TotalBuffer += Unitdata

            except socket.timeout:
                if self.recvChk:
                    continue
                else:
                    self.recvChk = False
                    self.get_logger().warn('Socket timeout, no data received')
                    break
            except Exception as e:
                self.get_logger().error(f'cam_image : {e}')

            time.sleep(0.01)

    def getImg(self):
        if self.TotalIMG is not None:
            # 이미지 데이터를 BGR 형식으로 변환하여 CvBridge를 사용해 ROS Image 메시지로 변환
            ros_img = self.bridge.cv2_to_imgmsg(self.TotalIMG, "bgr8")
            ros_img.header.frame_id = "velodyne"  # 프레임 ID 설정
            return ros_img
        else:
            return None

def main():
    # Initialize host, port, and destination port
    host = '127.0.0.1'  # Example host, change it accordingly
    port = 9190  # Example port, change it accordingly
    destination_port = 9191  # Example destination port, change it accordingly

    # Initialize CAMConnector instance with UDP network type
    rclpy.init(args=None)
    cam_connector = CAMConnector('UDP', destination_port)

    # Connect to the camera
    cam_connector.connect(host, port, destination_port)
    rclpy.spin(cam_connector)
    cam_connector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
