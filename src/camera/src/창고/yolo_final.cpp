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
#include "vision_msgs/msg/object_hypothesis_with_pose.h"
#include "std_msgs/msg/float32_multi_array.hpp"

#define A   7
#define B   8
#define AREA_SIZE 5000          //8000

using namespace std;
using namespace cv;

cv::Mat frame1;
cv::Mat sub_image1;

std::string Class_name;
std_msgs::msg::Bool A_flag_msg;
std_msgs::msg::Int16 flag_mission;
std_msgs::msg::Int32MultiArray treffic;
std_msgs::msg::Int32MultiArray mission_sign;

int obj_count;
int obj = 0;
int yolo_num[10][5];
int detect_length = 0;
bool detect = false;
int mission_lidar = 0;
int cam_port[10];
int cam_num = 0;
bool turn_on = false;
int i = 0;

float Max_Area = 0;
int mission = 0;
int count_mission = 0;
bool mission_stop = false;
int count_A1 = 0;
int count_A2 = 0;
int count_A3 = 0;
int count_B1 = 0;
int count_B2 = 0;
int count_B3 = 0;
// int sign[4]={0,0,0,0};
int red = 0;
int green = 0;
int greenleft = 0;
int uturn = 0;
int planning_clear = 0;
int planning_flag = 0;
// bool A_flag = false;
// bool speed_down = false;

bool loop_flag = false;
float Area = 0;
int dt = 0;
string b_stop = "not";

class Yolo : public rclcpp::Node{
    public:
    Yolo() : Node("yolo")       // 7, 9 ------> 10
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        // qos.transient_local();

        detection_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/Vision/flag_sign", qos_profile);
        traffic_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/Vision/traffic_sign",qos_profile);
        to_lidar_flag_pub = this->create_publisher<std_msgs::msg::Int16>("/Vision/mission_flag", qos_profile);

        // camera_port_number = this->create_subscription<std_msgs::msg::Int32MultiArray>("/Vision/camera_ports",1, std::bind(&Yolo::camera_port_callback, this,std::placeholders::_1));
        sub_detection = this->create_subscription<vision_msgs::msg::Detection2DArray>("/yolo_detect",qos_profile, std::bind(&Yolo::yolocallback, this,std::placeholders::_1));
        planning_mission_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", qos, std::bind(&Yolo::mission_cb, this, std::placeholders::_1));
        yolo_count_sub = this->create_subscription<std_msgs::msg::Int16>("/yolo_count", qos_profile, std::bind(&Yolo::YOLOCountCallback, this, std::placeholders::_1));
    }
    private:

        void yolocallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

        void mission_cb(const std_msgs::msg::Int16::SharedPtr msgs)
        {
            planning_clear = msgs->data;
            if (planning_flag != planning_clear)
            {
                planning_flag = planning_clear;
                if (planning_flag != 7 or planning_flag != 8 or planning_flag != 9 or planning_flag != 10)
                {
                    dt = 0;
                }
            }
            
        }

        void YOLOCountCallback(const std_msgs::msg::Int16::SharedPtr msg)
        {
            obj_count = msg->data;

            if (obj_count == 0)
            {
                cout << "-------------------------------------------------\n" << endl;
                cout << "Detect Nothing" << endl;

                count_A1 = 0;
                count_A2 = 0;
                count_A3 = 0;
                count_B1 = 0;
                count_B2 = 0;
                count_B3 = 0;
                mission = 0;
                red = 0;
                green = 0;
                greenleft = 0;
                uturn = 0;
                mission_lidar = 0;
                obj = 0;
                i = 0;
            }

            else{
                obj = 1;
            }

            mission_sign.data.push_back(obj_count);
            mission_sign.data.push_back(mission);
            detection_pub->publish(mission_sign);

            treffic.data.push_back(red);
            treffic.data.push_back(green);
            treffic.data.push_back(greenleft);
            treffic.data.push_back(uturn);
            traffic_pub->publish(treffic);

            flag_mission.data = mission_lidar;
            to_lidar_flag_pub->publish(flag_mission);

            cout << "-------------------------------------------------\n" << endl;
            cout << "Class_name [" << obj_count << "] : " << Class_name << endl;
            cout<<"[green, red, greenleft, uturn] : " << green << ", "<<red<<", "<<greenleft<<", "<<uturn<<endl;
            cout << "Area : " << Area << endl;
            cout<<"count_A == [ "<<count_A1<< ", " << count_A2<<", "<< count_A3<< " ]"<<endl;
            cout<<"count_B == [ "<<count_B1<< ", " << count_B2<<", "<< count_B3<< " ]"<<endl;
            cout << "object: "<<obj << "\tmission: " << mission << endl;
            cout<<"lidar_flag: " << mission_lidar<<endl;
            cout << "-------------------------------------------------\n" << endl;
            cout << "\n" << endl;
        }
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr detection_pub;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr traffic_pub;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr to_lidar_flag_pub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detection;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_sub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr yolo_count_sub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr planning_mission_sub;
        // rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr camera_port_number;
        rclcpp::TimerBase::SharedPtr timer_t;
};

void Yolo::yolocallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    b_stop = "B2";
    printf("Waiting for ---/yolov7/yolov7/visualization---\n");
    std_msgs::msg::Bool mission_stop_msg;
    std_msgs::msg::Bool mission_speed_down;
    std_msgs::msg::Int32MultiArray mission_sign;
    obj = 1;
    if (obj_count != 0)
    {
        std::string Class_name;

        for(int i=0; i<obj_count;i++)
        {
            for (const auto& detection : msg->detections)
            {
                Class_name = detection.results[0].id;

                yolo_num[i][1] = msg->detections[i].bbox.center.x - (msg->detections[i].bbox.size_x)/2;
                yolo_num[i][2] = msg->detections[i].bbox.center.x + (msg->detections[i].bbox.size_x)/2;
                yolo_num[i][3] = msg->detections[i].bbox.center.y - (msg->detections[i].bbox.size_y)/2;
                yolo_num[i][4] = msg->detections[i].bbox.center.y + (msg->detections[i].bbox.size_y)/2;

                float x = yolo_num[i][2]-yolo_num[i][1];
                float y = yolo_num[i][4]-yolo_num[i][3];

                Area = (yolo_num[i][2]-yolo_num[i][1])*(yolo_num[i][4]-yolo_num[i][3]);
                
                // if (dt == 0 && (planning_flag == 7 or planning_flag == 8 or planning_flag == 7 or planning_flag == 10 or))
                // {
                //     if (Class_name == "A1")
                //     {
                //         count_A1 += 1;
                //     }
                // }
                if (planning_flag == 10)
                {
                    if (Class_name == "green")
                    {
                        green = 1;
                    }
                    else if (Class_name == "red")
                    {
                        red = 1;
                    }
                    else if (Class_name == "greenleft")
                    {
                        greenleft = 1;
                    }
                }
                else if (planning_flag == 8)
                {
                    if (Class_name == "uturn")
                    {
                        uturn = 1;
                    }
                }
                else if (planning_flag == 7)
                {
                    if (dt == 0)
                    {
                        if (Class_name == "A1")
                        {
                            count_A1 += 1;
                        }
                        else if (Class_name == "A2")
                        {
                            count_A2 += 1;
                        }
                        else if (Class_name == "A3")
                        {
                            count_A3 += 1;
                        }

                    }
                    else
                    {
                        if (Class_name == "A1")
                        {
                            if (Area > AREA_SIZE) count_A1 += 1;
                        }
                        else if (Class_name == "A2")
                        {
                            if (Area > AREA_SIZE) count_A2 += 1;
                        }
                        else if (Class_name == "A3")
                        {
                            if (Area > AREA_SIZE) count_A3 += 1;
                        }
                    }
                }
                else if (planning_flag == 9)
                {
                    if (dt == 0)
                    {
                        if (Class_name == b_stop && Class_name == "B1")
                        {
                            count_B1 += 1;
                        }
                        else if (Class_name == b_stop && Class_name == "B2")
                        {
                            count_B2 += 1;
                        }
                        else if (Class_name == b_stop && Class_name == "B3")
                        {
                            count_B3 += 1;
                        }
                    }
                    else
                    {
                        if (Class_name == b_stop && Class_name == "B1")
                        {
                            if (Area > AREA_SIZE) count_B1 += 1;
                        }
                        else if (Class_name == b_stop && Class_name == "B2")
                        {
                            if (Area > AREA_SIZE) count_B2 += 1;
                        }
                        else if (Class_name == b_stop && Class_name == "B3")
                        {
                            if (Area > AREA_SIZE) count_B3 += 1;
                        }
                    }
                }
            }
        }
        i = i + 1;
        if (i == 5)
        {
            dt = 1;

            if (count_A1 < count_A3 && count_A2 < count_A3 && count_B1 < count_A3 && count_B2 < count_A3 && count_B3 < count_A3)
            {
                mission = 13;
                b_stop = "B3";
                if (dt != 0) mission_lidar=A;
            }
            else if (count_A1 < count_A2 && count_A3 < count_A2 && count_B1 < count_A2 && count_B2 < count_A2 && count_B3 < count_A2)
            {
                mission = 12;
                b_stop = "B2";
                if (dt != 0) mission_lidar=A;
            }
            else if (count_A3 < count_A1 && count_A2 < count_A1 && count_B1 < count_A1 && count_B2 < count_A1 && count_B3 < count_A1)
            {
                mission = 11;
                b_stop = "B1";
                if (dt != 0) mission_lidar=A;
            }
            else if(count_B1 < count_B3 && count_B2 < count_B3 && count_A1 < count_B3 && count_A2 < count_B3 && count_A3 < count_B3)
            {
                mission = 23;
                if (dt != 0) mission_lidar=B;
            }
            else if (count_B1 < count_B2 && count_B3 < count_B2 && count_A1 < count_B2 && count_A2 < count_B2 && count_A3 < count_B2)
            {
                mission = 22;
                if (dt != 0) mission_lidar=B;
            }
            else if (count_B3 < count_B1 && count_B2 < count_B1 && count_A1 < count_B1 && count_A2 < count_B1 && count_A3 < count_B1)
            {
                mission = 21;
                if (dt != 0) mission_lidar=B;
            }
            mission_sign.data.push_back(obj);
            mission_sign.data.push_back(mission);
            detection_pub->publish(mission_sign);

            treffic.data.push_back(red);
            treffic.data.push_back(green);
            treffic.data.push_back(greenleft);
            treffic.data.push_back(uturn);
            traffic_pub->publish(treffic);

            flag_mission.data = mission_lidar;
            to_lidar_flag_pub->publish(flag_mission);
            
            count_A1 = 0;
            count_A2 = 0;
            count_A3 = 0;
            count_B1 = 0;
            count_B2 = 0;
            count_B3 = 0;
            mission = 0;
            red = 0;
            green = 0;
            greenleft = 0;
            uturn = 0;
            // mission_lidar = 0;
            i = 0;

            mission_sign.data.clear();
            treffic.data.clear();
        }
        cout << "-------------------------------------------------\n" << endl;
        cout << "Detect Nothing" << endl;
        cout << "-------------------------------------------------\n" << endl;
        cout << "Class_name [" << obj_count << "] : " << Class_name << endl;
        
        cout<<"[green, red, greenleft, uturn] : " << green << ", "<<red<<", "<<greenleft<<", "<<uturn<<endl;
        cout << "Area : " << Area << endl;
        cout<<"count_A == [ "<<count_A1<< ", " << count_A2<<", "<< count_A3<< " ]"<<endl;
        cout<<"count_B == [ "<<count_B1<< ", " << count_B2<<", "<< count_B3<< " ]"<<endl;
        cout<<"lidar_flag: " << mission_lidar<<endl;
        cout << "-------------------------------------------------\n" << endl;
        cout << "\n" << endl;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Yolo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
