#include "lidar/LiDAR_parallel_parking.hpp"

using PointT = pcl::PointXYZI;
using namespace std;

LiDAR_parallel_parking::LiDAR_parallel_parking()
    : Mission("LiDAR_parallel_parking")
{
  velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/parallel", 1);
  object_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/parking_cen", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/parking_cen_mark", 1);
  marker_pub2_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/parallel_roi", 1);
  park_ok2_ = this->create_publisher<std_msgs::msg::Bool>("/LiDAR/park_ok2", 1);

  RCLCPP_INFO(this->get_logger(), "LiDAR_parallel_parking is ready");
}

int LiDAR_parallel_parking::get_status_code()
{
  return 14;
}

void LiDAR_parallel_parking::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  if (check_pointcloud == 0)
  {
    RCLCPP_INFO(this->get_logger(), "LiDAR_parallel_parking node sub pointcloud");
    check_pointcloud = 1;
  }
  // cout << "라이다 콜백" << endl;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *cloud);

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> VG;
  VG.setInputCloud(cloud);
  VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.filter(*cloud_filtered);

  pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_filtered);

  //////////////////////// 라이다 고정되면 수정 필요 /////////////////////////////////////////
  if (park_area == false)
  {
    cropFilter.setMin(Eigen::Vector4f(2.5, -6.5, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(3.5, -0.5, 0.5, 0.0));
    //[kcity]
    // cropFilter.setMin(Eigen::Vector4f(2.5, -5.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(3.5, -0.5, 0.5, 0.0));
  }

  else if (park_area == true && park_ok2 == false)
  {
    cropFilter.setMin(Eigen::Vector4f(3.0, -3.0, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(6.0, -0.5, 0.5, 0.0));
    //[kcity]
    // cropFilter.setMin(Eigen::Vector4f(3.0, -2.0, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(6.0, -0.5, 0.5, 0.0));
  }
  else if (park_ok2 == true)
  {
    cropFilter.setMin(Eigen::Vector4f(-0.2, -6.5, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(8.0, -0.5, 0.5, 0.0));
    //[kcity]
    // cropFilter.setMin(Eigen::Vector4f(-0.2, -5.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(8.0, -0.5, 0.5, 0.0));
  }

  ///////////////////////////////////////////////////////////////////////////////////////

  //////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
  // visualization_msgs::msg::Marker marker;
  // marker.header.frame_id = "velodyne";
  // marker.ns = "my_roi";
  // marker.id = 0;
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
  // marker_pub2_->publish(marker);
  /////////////////////////////////////////////////////////////////////////////////////

  cropFilter.filter(*cloud_filtered2);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered2);
  vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(cloud_filtered2);

  if (park_area == false)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }
  else if (park_area == true && park_ok2 == false)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }
  else if (park_ok2 == true)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }

  int ii = 0;
  pcl::PointCloud<PointT> TotalCloud;
  clusters.clear();
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
  {
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cluster->points.push_back(cloud_filtered2->points[*pit]);
      PointT pt = cloud_filtered2->points[*pit];
      PointT pt2;
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

  if (park_ok2 == false && park_ok3 == false)
  {
    roi_change();
  }

  else if (park_ok2 == true && park_ok3 == false)
  {
    if (ok2_count < 5)
    {
      park_ok2_msg.data = park_ok2;
      park_ok2_->publish(park_ok2_msg);
      ok2_count++;
    }
    if (ok2_count >= 5)
    {
      park_ok3 = true;
    }
  }

  else if (park_ok3 == true)
  {
    find_parking_cen();
  }
}

void LiDAR_parallel_parking::roi_change()
{
  // cout << "cluster.size(): " << clusters.size() << endl;
  if (clusters.size() > 0)
  {
    int start_cluster = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
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

      if (park_area == false)
      {
        ///// 첫 ROI 콘 사이즈 /////
        //////////////////////// 라이다 고정되면 수정 필요 /////////////////////////////////////////
        if ((max_point.y - min_point.y) < 0.20 && (max_point.y - min_point.y) > 0.05 && (max_point.z - min_point.z) > 0.2 && (max_point.z - min_point.z) < 0.6 && max_point.z < 0.1 && max_point.z > -0.3)
        ///////////////////////////////////////////////////////////////////////////////////////
        {
          start_cluster = start_cluster + 1; // 주차시작 부분을 감지하고 ROI 변경을 위한 부분
        }
        // cout << "start_cluster: " << start_cluster << endl;
      }
    }

    if (park_area == false && start_cluster >= 2)
    {
      std::cout << "check:2" << std::endl;
      park_area = true;
      park_ok1 = true;
    }
  }

  /////// 클러스터가 없으면 check3로 넘어가는데 가끔 클러스터가 있는데 사라지는 순간이 있어 방지하기 위해 추가 //////////
  if (park_area == true && clusters.size() == 0 && cluster_count <= 2)
  {
    cluster_count = cluster_count + 1;
    // std::cout << "cluster_count: " << cluster_count << std::endl;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  else if (park_area == true && cluster_count > 2)
  {
    std::cout << "check:3" << std::endl;
    park_ok2 = true;
  }
}

void LiDAR_parallel_parking::find_parking_cen()
{
  if (park_ok3)
  {
    detection2 = 1;
    if (detection != detection2)
    {
      std::chrono::seconds sleepDuration(3);
      std::this_thread::sleep_for(sleepDuration);
      detection = detection2;
    }
  }
  std::vector<geometry_msgs::msg::Point> point_list;
  if (pub_count <= 5)
  {
    // cout << "디버깅" << endl;
    if (clusters.size() > 0)
    {
      for (size_t i = 0; i < clusters.size(); i++)
      {
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

        float cen_x = center_point.x;
        float cen_y = center_point.y;

        geometry_msgs::msg::Point point_one;
        point_one.x = cen_x;
        point_one.y = cen_y;
        point_list.push_back(point_one);
      }
      size_t point_size = point_list.size();
      // cout << "size_check" << endl;
      // cout << "point_list.size(): " << point_size << endl;

      float minDistance = std::numeric_limits<float>::max();
      float minY = std::numeric_limits<float>::max();

      float x1_best; // 주차공간 좌측 두 콘의 x, y 좌표
      float y1_best; // 주차공간 좌측 두 콘의 x, y 좌표
      float x2_best; // 주차공간 좌측 두 콘의 x, y 좌표
      float y2_best; // 주차공간 좌측 두 콘의 x, y 좌표

      float x_cen1; // 주차공간 좌측 두 콘의 중심 x좌표 = 주차공간 중심 x좌표
      float y_cen1; // 주차공간 좌측 두 콘의 중심 y좌표

      float x_right;
      float y_right; // 주차공간 우측 한 콘의 y좌표

      float y_cen3; // 주차공간의 중심 y좌표

      float minXok; // 좌측 두 콘의 x좌표 사이에 있는 클러스터만 고려
      float minYok; // y값을 비교하여 저장하기 위함

      ////////////////// 주차 공간 왼쪽 라인 콘 두개 찾기(x) ////////////////////
      for (size_t i = 0; i < point_size - 1; i++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        Eigen::Vector3f scale_;
        pcl::compute3DCentroid(*clusters[i], centroid);
        pcl::getMinMax3D(*clusters[i], min_p, max_p);
        geometry_msgs::msg::Point center_point2;
        center_point2.x = centroid[0];
        center_point2.y = centroid[1];
        center_point2.z = centroid[2];
        geometry_msgs::msg::Point min_point2;
        min_point2.x = min_p[0];
        min_point2.y = min_p[1];
        min_point2.z = min_p[2];
        geometry_msgs::msg::Point max_point2;
        max_point2.x = max_p[0];
        max_point2.y = max_p[1];
        max_point2.z = max_p[2];

        minXok = point_list[i].x;
        minYok = point_list[i].y;

        for (size_t j = i + 1; j < point_size; j++)
        {
          float dx;
          float dy;
          float len;

          dx = point_list[i].x - point_list[j].x;
          dy = point_list[i].y - point_list[j].y;
          len = sqrt(dx * dx + dy * dy);

          // if (len >= 4.8 && len <= 5.2) // 주차공간 길이
          if (len >= 4.65 && len <= 5.35) // 주차공간 길이
          {
            float midY;
            float distanceAxis;

            midY = (point_list[i].y + point_list[j].y) / 2;

            distanceAxis = abs(midY);
            if (distanceAxis < minDistance) // 모든 점을 이어 중심점을 구한 뒤 중심점이 y축에 가까운 두 점을 업데이트 하여 저장
            {
              minDistance = distanceAxis;
              // x1_best = point_list[i].x;
              // y1_best = point_list[i].y;
              // x2_best = point_list[j].x;
              // y2_best = point_list[j].y;

              // 검출된 두 점을 나열해서 저장
              std::pair<float, float> minmax_x = std::minmax({point_list[i].x, point_list[j].x});
              float min_x = minmax_x.first;
              float max_x = minmax_x.second;

              std::pair<float, float> y_values;
              if (point_list[i].x < point_list[j].x)
              {
                y_values = std::make_pair(point_list[i].y, point_list[j].y);
              }
              else
              {
                y_values = std::make_pair(point_list[j].y, point_list[i].y);
              }
              float y1 = y_values.first;
              float y2 = y_values.second;

              x1_best = min_x;
              y1_best = y1;
              x2_best = max_x;
              y2_best = y2;

              x_cen1 = (x1_best + x2_best) / 2;
              y_cen1 = (y1_best + y2_best) / 2;
              // std::cout << "x1_best: " << x1_best << std::endl;
              // std::cout << "x2_best: " << x2_best << std::endl;
            }
          }
        }

        //////////////////// 아래부터 y좌표 찾기////////////////////
        int dist = 1.0;
        if (x1_best > x2_best)
        {
          if (minXok > x2_best + dist && minXok < x1_best - dist && minYok < y_cen1)
          {
            if (abs(minY) > abs(minYok))
            {
              minY = minYok;
              x_right = minXok;
              y_right = minYok;
            }
          }
        }

        else if (x1_best < x2_best)
        {
          if (minXok > x1_best + dist && minXok < x2_best - dist && minYok < y_cen1)
          {
            if (abs(minY) > abs(minYok))
            {
              minY = minYok;
              x_right = minXok;
              y_right = minYok;
            }
          }
        }

        obj.data.clear();
      }

      float incline;
      float angle;
      incline = (y2_best - y1_best) / (x2_best - x1_best);
      angle = std::abs(std::atan(incline));

      if (y_right < y_cen1 - 2.8 * cos(angle) && y_right > y_cen1 - 3.2 * cos(angle)) // 추출된 우측 콘의 y좌표가 정상적인 값인지 확인 후 주차공간 중심 y좌표 계산에 사용
      {
        std::cout << "success" << std::endl;
        y_cen3 = (y_cen1 + y_right) / 2;
      }

      else // 위 if문에 해당되지 않을 시 보험, 좌측 두 콘을 통해 기울기 고려
      {
        std::cout << "fail" << std::endl;
        y_cen3 = y_cen1 - 1.5 * cos(angle);
      }

////////////////////////////////////////////////////////////////////////////////
      if (x_cen1 > 6.0 || x_cen1 < 0.0 || y_cen3 > 0.0 || y_cen3 < -4.0 )
      {
        // 실험적으로 수정필요 //
        std::cout << "faillllllllllllllllllllllllllllllll" << std::endl;
        std::cout << "faillllllllllllllllllllllllllllllll" << std::endl;
        std::cout << "faillllllllllllllllllllllllllllllll" << std::endl;

        // x_cen1 = 3.91; // 4.9미터 일때 3.8 5.1미터 일때 4.1  4.95미터 일때 3.57  //1.42미터에서 앞쪽 콘 발견!!!!!!!!!!!!!!!!!!
        // y_cen3 = -2.55;
        // x_cen1 = 3.79;//마지막 실험값
        // y_cen3 = -2.60;
        x_cen1 = 3.25; // 0.75+2.5
        y_cen3 = -2.63;
      }
////////////////////////////////////////////////////////////////////////////////

      cout << "x: " << x_cen1 << endl;
      cout << "y: " << y_cen3 << endl;

      // cout << "x_right: " << x_right << endl;
      // cout << "y_right: " << y_right << endl;

      obj.data.push_back(x_cen1);
      obj.data.push_back(y_cen3);

      /////////////////// < 주차공간 중심을 나타내는 점 마커 > ///////////////
      // visualization_msgs::msg::Marker marker;
      // visualization_msgs::msg::MarkerArray markerArray;
      // geometry_msgs::msg::Point p;
      // marker.header.frame_id = "velodyne";
      // marker.ns = "my_marker";
      // marker.id = 9;
      // marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      // marker.action = visualization_msgs::msg::Marker::ADD;
      // marker.type = visualization_msgs::msg::Marker::POINTS;
      // marker.lifetime.nanosec = 100000000;
      // marker.scale.x = 0.1;
      // marker.scale.y = 0.1;
      // marker.color.r = 1.0;
      // marker.color.g = 0.0;
      // marker.color.b = 0.0;
      // marker.color.a = 1.0;
      // p.x = x_cen1;
      // p.y = y_cen3;
      // // p.x = x_right;
      // // p.y = y_right;
      // marker.points.push_back(p);
      // markerArray.markers.push_back(marker);
      // marker_pub_->publish(markerArray);
      //////////////////////////////////////////////////////////////////

      object_pub_->publish(obj);
      obj.data.clear();
    }
    pub_count = pub_count + 1;
  }
}
