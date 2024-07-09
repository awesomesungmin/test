#include "lidar/LiDAR_uturn.hpp"
#include "lidar/utils.hpp"


LiDAR_uturn::LiDAR_uturn()
: Mission("LiDAR_uturn")
{

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/uturn", 1);
    pub_center_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_point", 1);

    RCLCPP_INFO(this->get_logger(), "LiDAR_uturn is ready");
  
}

int LiDAR_uturn::get_status_code()
{
    return 16;
}

void LiDAR_uturn::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
      RCLCPP_INFO(this->get_logger(), "LiDAR_uturn node sub pointcloud");
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

    //===========================================[   R   O   I   영   역   ]=========================================================================

    // cropFilter.setMin(Eigen::Vector4f(0.0, -7.0, -0.3, 0)); // x, y, z, min (m)   
    // cropFilter.setMax(Eigen::Vector4f(18.0, 3.0, 1.5, 0)); // for kcity
    cropFilter.setMin(Eigen::Vector4f(0.0, -7.0, -0.3, 0)); // for school 
    cropFilter.setMax(Eigen::Vector4f(18.0, 3.0, 1.5, 0)); // for school

    cropFilter.filter(*cloud_filtered2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_erp(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<erpPoint> erpVec;
    pcl::PointIndices::Ptr erpIndices(new pcl::PointIndices);

    for (auto iter = cloud_filtered2->begin(); iter != cloud_filtered2->end(); ++iter)
    {
        if (iter->z > 0.5)
        {
        float x = iter->x;
        float y = iter->y;
        erpPoint erp;
        erp.x = x;
        erp.y = y;
        bool erppoint = false;

        for (auto it = erpVec.begin(); it != erpVec.end(); ++it)
        {
            if (abs(it->y - y) < 2 && abs(it->x - x) < 2)
            {
            erppoint = true;
            break;
            }
        }
        if (!erppoint)
            erpVec.push_back(erp);
        }
    }
    int i = 0;
    pcl::ExtractIndices<pcl::PointXYZI> ext;
    for (auto iter = cloud_filtered2->begin(); iter != cloud_filtered2->end(); ++iter)
    {
        for (auto it = erpVec.begin(); it != erpVec.end(); ++it)
        {
        float x = it->x;
        float y = it->y;

        if (abs(iter->y - y) < 2 && abs(iter->x - x) < 2)
        {
            iter->intensity = 100;
            erpIndices->indices.push_back(i);
            break;
        }
        }
        ++i;
    }

    ext.setInputCloud(cloud_filtered2);
    ext.setIndices(erpIndices);
    ext.setNegative(true);
    ext.filter(*cloud_erp);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_erp);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_erp);
    ec.setClusterTolerance(0.6);
    ec.setMinClusterSize(3);            //1010 jh
    // ec.setMinClusterSize(5);         // base
    // ec.setMaxClusterSize(2000); 
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    int ii = 0;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
        cluster->points.push_back(cloud_erp->points[*pit]);
        pcl::PointXYZI pt = cloud_erp->points[*pit];
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
        std_msgs::msg::Float64MultiArray cone_center;

    for (int i = 0; i < clusters.size(); i++)
    {
        Eigen::Vector4f centroid, min_p, max_p;
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

        float ob_size = sqrt(pow(max_point.x - min_point.x, 2) + pow(max_point.y - min_point.y, 2));
        float z_size = (max_point.z - min_point.z);
        if ((max_point.y-min_point.y) < 0.5)
        {
            cone_center.data.push_back(center_point.x);
            cone_center.data.push_back(center_point.y);
        }
    }
        pub_center_->publish(cone_center);
        cone_center.data.clear();
    
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";
    pub_->publish(output);
}