#include "lidar/LiDAR_big_static.hpp"

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;

LiDAR_big_static::LiDAR_big_static()
: Mission("LiDAR_big_static")
{

    velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/big_static", 1);
    object_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/bigcone_point", 1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/big_static_ROI", 1);
    marker_pub2_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/big_static_BIG_POINT", 1);
    marker_pub3_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/big_static_BIG_BOX", 1);

    RCLCPP_INFO(this->get_logger(), "LiDAR_big_static is ready");
  
}

int LiDAR_big_static::get_status_code()
{
    return 12;
}

void LiDAR_big_static::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
      RCLCPP_INFO(this->get_logger(), "LiDAR_big_static node sub pointcloud");
      check_pointcloud = 1;
    }
    // cout << "LiDAR data callback" << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> VG;
    VG.setInputCloud(cloud);
    VG.setLeafSize(0.1f, 0.1f, 0.1f);
    VG.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> cropFilter;
    cropFilter.setInputCloud(cloud_filtered);

    //////////////////////// 라이다 고정 시, 수정 필요 /////////////////////////////////////////
    // cropFilter.setMin(Eigen::Vector4f(0.0, -6.0, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(20.0, 6.0, 2.0, 0.0)); 
    cropFilter.setMin(Eigen::Vector4f(0.0, -4.0, -0.4, 0.0)); //cropFilter.setMin(Eigen::Vector4f(0.0, -4.0, -0.2, 0.0));
    cropFilter.setMax(Eigen::Vector4f(15.0, 4.0, 2.0, 0.0)); 
    ///////////////////////////////////////////////////////////////////////////////////////

    //////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
    // visualization_msgs::msg::Marker marker;
    // marker.header.frame_id = "velodyne";
    // marker.ns = "my_roi";
    // marker.id = 0; // ID는 0으로 설정 (하나의 마커만 사용)   
    // marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.type = visualization_msgs::msg::Marker::CUBE;
    // marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
    // marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
    // marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
    // marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
    // marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
    // marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    // marker.color.a = 0.3;
    // marker_pub_->publish(marker);  
    /////////////////////////////////////////////////////////////////////////////////////

    cropFilter.filter(*cloud_filtered2); 

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered2); 
    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered2);
    ec.setClusterTolerance(0.7);
    ec.setMinClusterSize(10); 
    // ec.setMaxClusterSize(3000); 
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    int ii = 0;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    clusters.clear();
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

    pcl::PCLPointCloud2 cloud_p; 
    pcl::toPCLPointCloud2(TotalCloud, cloud_p); 
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output); 
    output.header.frame_id = "velodyne";  
    velodyne_pub_->publish(output);

    find_big();
}

void LiDAR_big_static::find_big()
{
    if (clusters.size() > 0)
    {  
      for (size_t i = 0; i < clusters.size(); i++)
      {
        // cout << "find_big" << endl;
        Eigen::Vector4f centroid, min_p, max_p;
        Eigen::Vector3f scale_;
        pcl::compute3DCentroid(*clusters[i], centroid); 
        pcl::getMinMax3D(*clusters[i], min_p, max_p);
        geometry_msgs::msg::Point center_point; 
        center_point.x = centroid[0]; 
        center_point.y = centroid[1]; 
        center_point.z = centroid[2];
        geometry_msgs::msg::Point min_point; 
        min_point.x = min_p[0]; 
        min_point.y = min_p[1]; 
        min_point.z = min_p[2];
        geometry_msgs::msg::Point max_point; 
        max_point.x = max_p[0]; 
        max_point.y = max_p[1]; 
        max_point.z = max_p[2];

        float left_x = min_point.x;
        float left_y = max_point.y;
        float right_x = min_point.x;
        float right_y = min_point.y;

        float cen_x = center_point.x; 
        float cen_y = center_point.y;

        visualization_msgs::msg::Marker marker; 
        visualization_msgs::msg::MarkerArray markerArray;
        
        //////////////////////// 라이다 고정되면 수정 필요 //////////////////////////////////
        if ((max_point.y - min_point.y) < 3.0 && (max_point.y - min_point.y) > 0.8  && (max_point.z-min_point.z) > 0.1  && (max_point.z-min_point.z) < 1.6)
        ///////////////////////////////////////////////////////////////////////////////
        {
          obj.data.push_back(left_x);
          obj.data.push_back(left_y);
          obj.data.push_back(right_x);
          obj.data.push_back(right_y);
          float xxx = (left_x + right_x) / 2;
          float yyy = (left_y + right_y) / 2;

          // obj.data.push_back(cen_x);
          // obj.data.push_back(cen_y);
          std::cout << "큰놈발견" << std::endl;
          std::cout << "x: " << cen_x << std::endl;
          std::cout << "y: " << cen_y << std::endl;

          ////////////////// < 장애물 센터 포인트를 나타내는 점 마커 > ////////////////////////
          geometry_msgs::msg::Point p;
          marker.header.frame_id = "velodyne";
          marker.ns = "my_marker";
          marker.id = i;
          marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.type = visualization_msgs::msg::Marker::POINTS;
          marker.lifetime.nanosec = 100000000;
          marker.scale.x = 0.1;
          marker.scale.y = 0.1;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          // p.x = cen_x;
          // p.y = cen_y;
          p.x = xxx;
          p.y = yyy;
          marker.points.push_back(p);
          markerArray.markers.push_back(marker);
          marker_pub2_->publish(markerArray);
          ////////////////////////////////////////////////////////////////////////////

          //////////////////// < 장애물 바운딩 박스 나타내는 마커 > ///////////////////////////
          // visualization_msgs::msg::Marker marker3;
          // visualization_msgs::msg::MarkerArray markerArray3;
          // marker3.header.frame_id = "velodyne";
          // marker3.ns = "my_marker";
          // marker3.id = i;
          // marker3.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          // marker3.action = visualization_msgs::msg::Marker::ADD;
          // marker3.type = visualization_msgs::msg::Marker::CUBE;
          // marker3.lifetime.nanosec = 100000000;
          // marker3.scale.x = width;
          // marker3.scale.y = height;
          // marker3.color.r = 0.5;
          // marker3.color.g = 0.5;
          // marker3.color.b = 0.5;
          // marker3.color.a = 0.5;
          // marker3.pose.orientation = quaternion;
          // marker3.pose.position = center_point;
          // markerArray3.markers.push_back(marker3);
          // marker_pub3_->publish(markerArray3);
          // markerArray3.markers.clear();
          //////////////////////////////////////////////////////////////////////////////
        }
      }
      if (!obj.data.empty())
      {
        object_pub_->publish(obj);
        obj.data.clear();
      }
    }
}
