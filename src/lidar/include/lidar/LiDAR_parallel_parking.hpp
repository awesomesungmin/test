#include "mission.hpp"


class LiDAR_parallel_parking : public Mission
{
public:
    LiDAR_parallel_parking();

private:
    //func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;
    int get_status_code() override;
    void roi_change();
    void find_parking_cen();
    
    //pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr object_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr park_ok2_;

    //var
    int check_pointcloud = 0;
    std_msgs::msg::Float64MultiArray obj;
    std_msgs::msg::Bool park_ok2_msg;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;

    bool park_area = false;
    bool park_ok1 = false;
    bool park_ok2 = false; 
    bool park_ok3 = false;
    int cluster_count = 0;
    int pub_count = 0;
    int detection = 0;
    int detection2 = 0;
    int ok2_count = 0; 
};
