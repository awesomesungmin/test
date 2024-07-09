/*
Example.hpp를 보고 왔나요~?
그럼 지금부터는 hpp에서 선언한 클래스를 가지고 코드를 작성해보아요

해석하기 싫으면 아래 구성 그대로 갖다 쓰면 오케이
*/ 

#include "lidar/Example.hpp"

Example::Example()
: Mission("Example")
{

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("필요한 곳으로 알아서", 1);

    RCLCPP_INFO(this->get_logger(), "Example is ready");
  
}

int Example::get_status_code()
{
    return 1999; // 꼭꼭꼭꼭꼭꼭꼭 수정하기!!!!!!
}

void Example::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 여기 부터는 코드 설명
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "lidar/Example.hpp"
// 클래스 구조를 선언한 hpp를 추가합니다.

Example::Example()
: Mission("Example") // <= "여기는 노드 이름을 넣어주세요"
{

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("필요한 곳으로 알아서", 1);

    RCLCPP_INFO(this->get_logger(), "Example is ready");
  
}

// 이 함수는 미션 번호 확인을 위한 함수니까, 꼭 미션에 맞게 바꿔주세요
int Example::get_status_code()
{
    return 1999;
}

// 이 함수는 라이다 콜백함수에요. 여기다가 코드 작성하면 됩니다.
void Example::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{

}

/*
만일 추가로 함수를 만들고 싶으면 

Example.hpp에 함수원형을 적어주고 여기다가 써주면 돼요

예시로 int Example::add_num(const std_msgs::msg::Int16::Shared msg) {} 라는 함수를
만들고 싶다면 hpp로 가서
int Example::add_num(const std_msgs::msg::Int16::Shared msg);
위와 같이 추가해준 다음
아래 처럼 써주면 돼요!!
*/

/*
int Example::add_num(const std_msgs::msg::Int16::Shared msg)
{
    int a = msg->data;

    int b = a + a;

    return b;
}
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 이제 클래스 구조 마스터!!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////