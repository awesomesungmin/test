#include "opencv2/opencv.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sstream>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <chrono>
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp" // 길요한이 수정
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;
using namespace cv;

std::string Class_name;
std_msgs::msg::Int32MultiArray traffic;

int obj_count;
int obj = 0;
int yolo_num[10][5];
int red = 0;
int green = 0;
int left_ = 0;
int orange = 0;
int planning_clear = 0;
int planning_flag = 0;
float Area = 0;
float traffic_area[5] = {0, 0, 0, 0, 0};
int traffic_detection[5] = {0, 0, 0, 0, 0};

float final_area = 0;
int final_detection = 0;

class Yolo : public rclcpp::Node{
public:
    Yolo() : Node("yolo")       // 7, 9 ------> 10
    {
        // rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)); // 길요한이 수정
        // qos.best_effort();
        // qos.transient_local();

        traffic_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/Vision/traffic_sign",1);

        // camera_port_number = this->create_subscription<std_msgs::msg::Int32MultiArray>("/Vision/camera_ports",1, std::bind(&Yolo::camera_port_callback, this,std::placeholders::_1));
        sub_detection = this->create_subscription<vision_msgs::msg::Detection2DArray>("/yolo_detect",1, std::bind(&Yolo::yolocallback, this,std::placeholders::_1));
        planning_mission_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", 1, std::bind(&Yolo::mission_cb, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&Yolo::timer_callback, this));
    }
private:

    int mission_check = -1; // 미션 들어오는 확인하는 변수
    int mission_count = 0; // 몇 번 동안 띄울지 결정하는 함수

    void yolocallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

    void timer_callback();

    void mission_cb(const std_msgs::msg::Int16::SharedPtr msgs)
    {
        // cout << "a" <<endl;
        planning_flag = msgs->data;
        mission_check = planning_flag;
        // planning_clear = msgs->data;
        // if (planning_flag != planning_clear) planning_flag = planning_clear;
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr traffic_pub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detection;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr planning_mission_sub;
    // rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr camera_port_number;
    rclcpp::TimerBase::SharedPtr timer_;
};

void Yolo::timer_callback()
{
    if (mission_check == -1)
    {
        mission_count = 0;
        RCLCPP_INFO(this->get_logger(), "\nerror : 🚨🚨 미션 어딨어? 미션 어딨어? 🚨🚨");
    }
    else if ( (mission_check == 21 || mission_check == 22) && mission_count < 1)
    {
        RCLCPP_INFO(this->get_logger(), "\n미션 간다 ~!🥇 미션 간다 ~!🥇");
        mission_count = 1;
    }
    mission_check = -1;
}

void Yolo::yolocallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    // b_stop ="B2"
    printf("Waiting for ---/yolov7/yolov7/visualization---\n");
    std::string Class_name;

    for(int i=0; i<msg->detections.size();i++)
    {
        for (const auto& detection : msg->detections)
        {
            Class_name = msg->detections[i].results[0].hypothesis.class_id; // 길요한이 수정
            yolo_num[i][1] = msg->detections[i].bbox.center.position.x - (msg->detections[i].bbox.size_x)/2.0; // 길요한이 수정
            yolo_num[i][2] = msg->detections[i].bbox.center.position.x + (msg->detections[i].bbox.size_x)/2.0; // 길요한이 수정
            yolo_num[i][3] = msg->detections[i].bbox.center.position.y - (msg->detections[i].bbox.size_y)/2.0; // 길요한이 수정
            yolo_num[i][4] = msg->detections[i].bbox.center.position.y + (msg->detections[i].bbox.size_y)/2.0; // 길요한이 수정

            float x = yolo_num[i][2]-yolo_num[i][1];
            float y = yolo_num[i][4]-yolo_num[i][3];

            Area = (yolo_num[i][2]-yolo_num[i][1])*(yolo_num[i][4]-yolo_num[i][3]);
            
            if ((planning_flag == 21) || (planning_flag == 22))
            {
                if (Class_name == "red")
                {
                    traffic_detection[i] = 1;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "orange")
                {
                    traffic_detection[i] = 2;
                    traffic_area[i] = Area;
                }
                else if ((Class_name == "green"))
                {
                    traffic_detection[i] = 3;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "left")
                {
                    traffic_detection[i] = 4;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "left&green")
                {
                    traffic_detection[i] = 5;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "red&left")
                {
                    traffic_detection[i] = 6;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "red&orange")
                {
                    traffic_detection[i] = 7;
                    traffic_area[i] = Area;
                }
                else if (Class_name == "orange&left")
                {
                    traffic_detection[i] = 8;
                    traffic_area[i] = Area;
                }
            }
        }
    }

    for(int i=0; i<msg->detections.size();i++)
    {
        if (final_area < traffic_area[i])
        {
            final_area = traffic_area[i];
            final_detection = traffic_detection[i];
        }
    }

    if (final_detection == 1)
    {
        red = 1;
    }
    else if (final_detection == 2)
    {
        orange = 1;
    }
    else if (final_detection == 3)
    {
        green = 1;
    }
    else if (final_detection == 4)
    {
        left_ = 1;
    }
    else if (final_detection == 5)
    {
        left_ = 1;
        green = 1;
    }
    else if (final_detection == 6)
    {
        left_ = 1;
        red = 1;
    }
    else if (final_detection == 7)
    {
        red = 1;
        orange = 1;
    }
    else if (final_detection == 8)
    {
        left_ = 1;
        orange = 1;
    }

    // cout << "-------------------------------------------------\n" << endl;
    // cout << "Detect Nothing" << endl;
    cout << "-------------------------------------------------\n" << endl;
    cout << "Class_name [" << obj_count << "] : " << Class_name << endl;
    cout<<"[red, orange, left_, green] : " << red << ", "<<orange<<", "<<left_<<", "<<green<<endl;
    cout << "Area : " << Area << endl;
    cout << "-------------------------------------------------\n" << endl;
    cout << "\n" << endl;

    traffic.data.push_back(red);
    traffic.data.push_back(orange);
    traffic.data.push_back(left_);
    traffic.data.push_back(green);
    traffic_pub->publish(traffic);
    
    final_area = 0;
    final_detection = 0;
    green = 0;
    red = 0;
    left_ = 0;
    orange = 0;
    for(int i=0; i<msg->detections.size();i++)
    {
        traffic_area[i] = 0;
        traffic_detection[i] = 0;
    }

    traffic.data.clear();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Yolo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
