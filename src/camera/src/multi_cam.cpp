#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

Mat frame1,frame2;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("multi_cam");

  int CameraNum1;
  int CameraNum2;

  // node->declare_parameter("CameraNum", 0);
  // CameraNum = node->get_parameter("CameraNum").as_int();
  
  cout << "연결할 카메라 번호를 쓰세요1" << endl;
  cin >> CameraNum1;

  cout << "연결할 카메라 번호를 쓰세요2" << endl;
  cin >> CameraNum2;

  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  auto publisher1 = node->create_publisher<sensor_msgs::msg::Image>("video1", rclcpp::SensorDataQoS());
  sensor_msgs::msg::Image::SharedPtr msg1;

  auto publisher2 = node->create_publisher<sensor_msgs::msg::Image>("video2", rclcpp::SensorDataQoS());
  sensor_msgs::msg::Image::SharedPtr msg2;

  VideoCapture cap1(CameraNum1);
  cap1.set(CAP_PROP_FRAME_WIDTH, 640);
  cap1.set(CAP_PROP_FRAME_HEIGHT, 480);

  VideoCapture cap2(CameraNum2);
  cap2.set(CAP_PROP_FRAME_WIDTH, 640);
  cap2.set(CAP_PROP_FRAME_HEIGHT, 480);
    
  rclcpp::WallRate loop_rate(20.0);

  while(rclcpp::ok())
  {
    cap1 >> frame1;
    cap2 >> frame2;



    msg1 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame1).toImageMsg();
    publisher1->publish(*msg1.get());

    msg2 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame2).toImageMsg();
    publisher2->publish(*msg2.get()); 


    rclcpp::spin_some(node);
    loop_rate.sleep();

    imshow("video1", frame1);
    imshow("video2", frame2);


    char ch = cv::waitKey(10);
    if(ch == 27) break;
  }
      
    
  return 0;

}