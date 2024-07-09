/*
지금부터 클래스 구조를 만들건데 해석을 건너뛰고 싶다면
아래 코드 그대로 가져가면 돼요.
쓰다가 모르면 밑에 설명도 있어요~
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mission.hpp"

class Example : public Mission
{
public:
    Example();   

private:
    //func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override; // 수정 X
    int get_status_code() override; // 수정 X

    //pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr other_type_pub_;
    
    //var
    int check_pointcloud = 0; // 수정 X

};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 여기 부터는 코드 설명
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 기본적인 헤더랑 스레드 관련 코드가 포함되어 있으니까 꼭 헤더 추가해주세요!!
#include "mission.hpp"

// 다음은 클래스 구조 작성하는 예시입니다.

// 예시)클래스 명 => Example
// Mission이라는 클래스를 상속 받습니다. Mission 클래스는 ros의 Node를 상속받은 상태입니다.
class Example : public Mission
{
public:
    // 여긴 클래스명만 쓰고 넘어가주세요
    // public에는 되도록이면 이정도만 적어주세요
    Example();   

private:
    //func 여긴 함수만 작성해주세요
    // 아래 두 함수는 각각 라이다 콜백과 미션 콜백을 수행하니까 수정할 필요 없어요
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override; // 수정 X
    int get_status_code() override; // 수정 X
    // 여기 이후에 추가 "함수 원형" 작성해주세요

    //pub 여긴 pub/sub 관련된 것만 적어주세요
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr other_type_pub_;
    
    //var 여긴 변수 관련해서 작성해주세요
    int check_pointcloud = 0; // 라이다 콜백 여부 확인하는 변수니 지우지 마세요

}; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // 끝~~~~~~~참 쉽죠?
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


