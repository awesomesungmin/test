#include "lidar/LiDAR_dynamic_tunnel.hpp"
#include "lidar/utils.hpp"

LiDAR_dynamic_tunnel::LiDAR_dynamic_tunnel()
: Mission("LiDAR_dynamic_tunnel")
{
  pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/dynamic_tunnel", 1);
  dynamic_stop_ = this->create_publisher<std_msgs::msg::Bool>("/LiDAR/dynamic_stop", 1); // 플래닝에게 줄 동적 stop

  RCLCPP_INFO(this->get_logger(), "LiDAR_dynamic_tunnel is ready");

}

int LiDAR_dynamic_tunnel::get_status_code()
{
    return 17;
}

void LiDAR_dynamic_tunnel::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
      RCLCPP_INFO(this->get_logger(), "LiDAR_dynamic_tunnel node sub pointcloud");
      check_pointcloud = 1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::VoxelGrid<pcl::PointXYZI> VG;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    VG.setInputCloud(cloud);
    VG.setLeafSize(0.1f, 0.1f, 0.1f);
    VG.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> cropFilter;
    cropFilter.setInputCloud(cloud_filtered);

    //====================[ 수정 할 곳 ]====================  R O I 영 역

    // 동적 ROI 설정
    cropFilter.setMin(Eigen::Vector4f(-0, -5, -0.5, 0));
    cropFilter.setMax(Eigen::Vector4f(15, 5, 1.0, 0));

    //=======================================================

    cropFilter.filter(*cloud_filtered2);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered2);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered2);

    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    int ii = 0;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
          cluster->points.push_back(cloud_filtered2->points[*pit]);
          pcl::PointXYZI pt = cloud_filtered2->points[*pit];
          pcl::PointXYZI pt2;
          pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
          pt2.intensity = (float)(ii + 1);
          TotalCloud.push_back(pt2);
      }
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }
            
    find_dynamic(TotalCloud);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";
    pub->publish(output);
}

void LiDAR_dynamic_tunnel::find_dynamic(const pcl::PointCloud<pcl::PointXYZI> &TotalCloud)
{
    
    MyPoint p1(M_first_x, M_first_y), p2(M_second_x, M_second_y), p3(M_third_x, M_third_y), p4(M_fourth_x, M_fourth_y);
    bool detected = false;
    MyPoint obs(0, 0);

    for (int i = 0; i < int(TotalCloud.points.size()); i++)
    {
        obs.setCoord(TotalCloud.points[i].x, TotalCloud.points[i].y);
        int count = 0;
        count += ccw(p2, p3, obs);
        count += ccw(p3, p4, obs);
        count += ccw(p4, p1, obs);
        if (count == 3 && TotalCloud.points[i].z > 0.0)
            detected = true;
    }
    if (detected)
    {
        std::cout << "[Object] Object!! " << std::endl;
    }
    else
    {
        std::cout << "[Objecet] No Object !!" << std::endl;
    }

    std_msgs::msg::Bool detected_;
    detected_.data = detected;
    dynamic_stop_->publish(detected_);
}