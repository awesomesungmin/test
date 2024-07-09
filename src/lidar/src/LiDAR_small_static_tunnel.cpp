#include "lidar/LiDAR_small_static_tunnel.hpp"

LiDAR_small_static_tunnel::LiDAR_small_static_tunnel()
: Mission("LiDAR_small_static_tunnel")
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/small_static_tunnel", 1);
  object_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen", 1);
  marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/small_static_tunnel_object",1);

  RCLCPP_INFO(this->get_logger(), "LiDAR_small_static_tunnel is ready");

}

int LiDAR_small_static_tunnel::get_status_code()
{
    return 15;
}

void LiDAR_small_static_tunnel::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
      RCLCPP_INFO(this->get_logger(), "LiDAR_small_static_tunnel node sub pointcloud");
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

    cropFilter.setMin(Eigen::Vector4f(0, -2, -0.3, 0)); // 수평 라이다
    cropFilter.setMax(Eigen::Vector4f(11, 4, 1.0, 0));

    //=======================================================clustering================= 
    
    cropFilter.filter(*cloud_filtered2);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered2); 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered2);
      
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3); 
    ec.setMaxClusterSize(2000); 
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
    //==========================================================point cloud 색깔 입히는 과정================
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

    std_msgs::msg::Float64MultiArray obj;
    std_msgs::msg::Float32MultiArray p;
    for (int i = 0; i < int(clusters.size()); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_p, max_p); 
      
      geometry_msgs::msg::Point center_point; 
      center_point.x = centroid[0]; center_point.y = centroid[1]; center_point.z = centroid[2];
      
      geometry_msgs::msg::Point min_point; 
      min_point.x = min_p[0]; min_point.y = min_p[1]; min_point.z = min_p[2];
      
      geometry_msgs::msg::Point max_point; 
      max_point.x = max_p[0]; max_point.y = max_p[1]; max_point.z = max_p[2];
      
      float cen_x = center_point.x; 
      float cen_y = center_point.y; 

      if ((max_point.y - min_point.y) < 1.5 && (max_point.y - min_point.y) > 0.5 && max_point.z < 1.0)
      {
        obj.data.push_back(cen_x);
        obj.data.push_back(cen_y);
        p.data.push_back(cen_x);
        p.data.push_back(cen_y);  
      }
    }

    obj.data.push_back(-1000);
    object_->publish(obj);
    obj.data.clear();

    p.data.push_back(-1000);
    p.data.clear();

    visualization_msgs::msg::MarkerArray p_array;

    geometry_msgs::msg::Point obj1;
    visualization_msgs::msg::Marker marker1; 
    marker1.ns  ="points_and_lines"; 
    marker1.action = visualization_msgs::msg::Marker::ADD; 
    marker1.type = visualization_msgs::msg::Marker::POINTS;
    marker1.id = 0; 
    marker1.pose.orientation.w = 1.0; marker1.scale.x = 0.5; marker1.scale.y = 0.5; 
    marker1.color.a = 1.0; marker1.color.r = 1.0f;
    obj1.x = p.data[0]; obj1.y = p.data[1]; obj1.z = 0.0; 
    marker1.points.push_back(obj1);
    marker1.header.frame_id = "velodyne";
    p_array.markers.push_back(marker1);

    geometry_msgs::msg::Point obj2;
    visualization_msgs::msg::Marker marker2; 
    marker2.ns  ="points_and_lines"; 
    marker2.action = visualization_msgs::msg::Marker::ADD; 
    marker2.type = visualization_msgs::msg::Marker::POINTS;
    marker2.id = 1; 
    marker2.pose.orientation.w = 1.0; marker2.scale.x = 0.5; marker2.scale.y = 0.5; 
    marker2.color.a = 1.0; marker2.color.g = 1.0f; 
    obj2.x = p.data[2]; obj2.y = p.data[3]; obj2.z = 0.0; 
    marker2.points.push_back(obj2);
    marker2.header.frame_id = "velodyne";
    p_array.markers.push_back(marker2);

    marker_array_->publish(p_array);

    pcl::PCLPointCloud2 cloud_p; 
    pcl::toPCLPointCloud2(TotalCloud, cloud_p); 
    sensor_msgs::msg::PointCloud2 output; 
    pcl_conversions::fromPCL(cloud_p, output); 
    output.header.frame_id = "velodyne"; 
    pub_->publish(output);

}
