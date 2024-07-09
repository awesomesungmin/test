#include "mission.hpp"


class LiDAR_dynamic_tunnel : public Mission
{
public:
    LiDAR_dynamic_tunnel();

private:
    //func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;
    int get_status_code() override;
    void find_dynamic(const pcl::PointCloud<pcl::PointXYZI> &TotalCloud);

    //pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dynamic_stop_;

    //var
    int check_pointcloud = 0;
    ////////////////////////////// 동적 영역/////////////////////////////////////
    float M_first_x = 0.0, M_first_y = 1.0;
    float M_second_x = 0.0, M_second_y = -1.0;
    float M_third_x = 4.0, M_third_y = -1.0;
    float M_fourth_x = 4.0, M_fourth_y = 1.0;
    ///////////////////////////////////////////////////////////////////////////

};