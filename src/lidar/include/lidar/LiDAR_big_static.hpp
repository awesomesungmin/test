#include "mission.hpp"


class LiDAR_big_static : public Mission
{
public:
    LiDAR_big_static();   

private:
    //func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;
    int get_status_code() override;
    void find_big();

    //pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr object_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub2_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub3_;
    
    //var
    int check_pointcloud = 0;
    std_msgs::msg::Float64MultiArray obj;

    
};

