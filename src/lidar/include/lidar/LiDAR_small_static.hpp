#include "mission.hpp"


class LiDAR_small_static : public Mission
{
public:
    LiDAR_small_static();

private:
    //func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;
    int get_status_code() override;
    
    //pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr object_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_;

    //var
    int check_pointcloud = 0;

};
