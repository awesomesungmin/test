#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <thread>
#include <pthread.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"

using namespace std;
using namespace cv;

struct Box
{
  float x;
  float y;
  float z;

  float size_x;
  float size_y;
  float size_z;
};

struct Box_yolo
{
  float x1;
  float x2;
  float y1;
  float y2;
  int color;
};

struct Point3D {
    float x;
    float y;
    float z;
};

struct Box_points {
    Point3D points[8];
};

Box_yolo transformBox(const cv::Mat& transformMat, const Box_points& real_points, float offsetY = 0.0f) {
    std::vector<float> xs(8), ys(8);
    
    for (int i = 0; i < 8; ++i) {
        double box[4] = {real_points.points[i].x, real_points.points[i].y, real_points.points[i].z, 1.0};
        cv::Mat pos(4, 1, CV_64F, box);

        cv::Mat newPos(transformMat * pos);
        
        xs[i] = (float)(newPos.at<double>(0, 0) / newPos.at<double>(2, 0));
        ys[i] = (float)(newPos.at<double>(1, 0) / newPos.at<double>(2, 0)) + offsetY;
    }

    return { 
      *std::min_element(xs.begin(), xs.end()), 
      *std::max_element(xs.begin(), xs.end()), 
      *std::min_element(ys.begin(), ys.end()), 
      *std::max_element(ys.begin(), ys.end()) };
}

Box_points calcBox_points(const Box &box)
{
  Box_points real_points;

  real_points.points[0].x = box.x - (box.size_x)/2;
  real_points.points[0].y = box.y + (box.size_y)/2;
  real_points.points[0].z = box.z - (box.size_z)/2;

  real_points.points[1].x = box.x - (box.size_x)/2;
  real_points.points[1].y = box.y - (box.size_y)/2;
  real_points.points[1].z = box.z - (box.size_z)/2;

  real_points.points[2].x = box.x - (box.size_x)/2;
  real_points.points[2].y = box.y - (box.size_y)/2;
  real_points.points[2].z = box.z + (box.size_z)/2;

  real_points.points[3].x = box.x - (box.size_x)/2;
  real_points.points[3].y = box.y + (box.size_y)/2;
  real_points.points[3].z = box.z + (box.size_z)/2;

  real_points.points[4].x = box.x + (box.size_x)/2;
  real_points.points[4].y = box.y + (box.size_y)/2;
  real_points.points[4].z = box.z - (box.size_z)/2;

  real_points.points[5].x = box.x + (box.size_x)/2;
  real_points.points[5].y = box.y - (box.size_y)/2;
  real_points.points[5].z = box.z - (box.size_z)/2;

  real_points.points[6].x = box.x + (box.size_x)/2;
  real_points.points[6].y = box.y - (box.size_y)/2;
  real_points.points[6].z = box.z + (box.size_z)/2;

  real_points.points[7].x = box.x + (box.size_x)/2;
  real_points.points[7].y = box.y + (box.size_y)/2;
  real_points.points[7].z = box.z + (box.size_z)/2;

  return real_points;
}

float get_iou(const Box_yolo &a, const Box_yolo &b)
{
  float x_left = max(a.x1, b.x1);
  float y_top = max(a.y1, b.y1);
  float x_right = min(a.x2, b.x2);
  float y_bottom = min(a.y2, b.y2);

  if (x_right < x_left || y_bottom < y_top)
    return 0.0f;

  float intersection_area = (x_right - x_left) * (y_bottom - y_top);

  float area1 = (a.x2 - a.x1) * (a.y2 - a.y1);
  float area2 = (b.x2 - b.x1) * (b.y2 - b.y1);

  float iou = intersection_area / (area1 + area2 - intersection_area);

  return iou;
}

std::mutex mut_img, mut_pc, mut_box;
std::vector<Box> boxes;
std::vector<Box_yolo> boxes_yolo;

std::mutex mut_yolo;
std::string Class_name;

using Stamped3PointMsg = vision_msgs::msg::Detection3DArray;
using Stamped3PointMsgSubscriber = message_filters::Subscriber<Stamped3PointMsg>;

using Stamped2PointMsg = vision_msgs::msg::Detection2DArray;
using Stamped2PointMsgSubscriber = message_filters::Subscriber<Stamped2PointMsg>;

using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<Stamped3PointMsg, Stamped2PointMsg>;
using ApproximateSync = message_filters::Synchronizer<ApproximateSyncPolicy>;

class ImageLiDARFusion : public rclcpp::Node
{
public:
  vector<double> CameraExtrinsic_vector_right;
  vector<double> CameraExtrinsic_vector_left;
  vector<double> CameraMat_vector_right;
  vector<double> CameraMat_vector_left;
  vector<double> DistCoeff_vector_right;
  vector<double> DistCoeff_vector_left;

  Mat transformMat_right;
  Mat transformMat_left;
  Mat image_undistorted;

  bool is_rec_image = false;
  bool is_rec_yolo = false;
  bool is_rec_box = false;

  int box_locker = 0;
  int yolo_locker = 0;
  float min_area = 100.;
  float min_iou = 0.2;

public:
  ImageLiDARFusion()
  : Node("amz_compare_box")
  {
    RCLCPP_INFO(this->get_logger(), "----------- initialize -----------\n");

    this->declare_parameter("CameraExtrinsicMat_right", vector<double>());
    this->CameraExtrinsic_vector_right = this->get_parameter("CameraExtrinsicMat_right").as_double_array();
    this->declare_parameter("CameraMat_right", vector<double>());
    this->CameraMat_vector_right = this->get_parameter("CameraMat_right").as_double_array();
    this->declare_parameter("DistCoeff_right", vector<double>());
    this->DistCoeff_vector_right = this->get_parameter("DistCoeff_right").as_double_array();   

    this->declare_parameter("CameraExtrinsicMat_left", vector<double>());
    this->CameraExtrinsic_vector_left = this->get_parameter("CameraExtrinsicMat_left").as_double_array();
    this->declare_parameter("CameraMat_left", vector<double>());
    this->CameraMat_vector_left = this->get_parameter("CameraMat_left").as_double_array();
    this->declare_parameter("DistCoeff_left", vector<double>());
    this->DistCoeff_vector_left = this->get_parameter("DistCoeff_left").as_double_array();   

    this->set_param();

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 1;
    custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    yolo_detect_sub_ = std::make_shared<Stamped2PointMsgSubscriber>(this, "/yolo_detect", custom_qos_profile);
    LiDAR_sub_ = std::make_shared<Stamped3PointMsgSubscriber>(this, "/lidar_bbox", custom_qos_profile);

    approximate_sync_ = std::make_shared<ApproximateSync>(ApproximateSyncPolicy(1), *LiDAR_sub_, *yolo_detect_sub_);
    approximate_sync_->registerCallback(std::bind(&ImageLiDARFusion::approximateSyncCallback, this, std::placeholders::_1, std::placeholders::_2));

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_LiDAR", 1);
    cone_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_color", 1);

    auto timer_callback = [this]() -> void{ FusionCallback(); };
    timer_ = create_wall_timer(33ms, timer_callback);

    RCLCPP_INFO(this->get_logger(), "------------ intialize end------------\n");
  }

  ~ImageLiDARFusion(){}

  void set_param();
  void FusionCallback();

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cone_pub_;

  std::shared_ptr<Stamped3PointMsgSubscriber> LiDAR_sub_;
  std::shared_ptr<Stamped2PointMsgSubscriber> yolo_detect_sub_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  rclcpp::TimerBase::SharedPtr timer_;
  int sync_locker = 0;

  void approximateSyncCallback(
    const std::shared_ptr<const vision_msgs::msg::Detection3DArray>& lidar_msg,
    const std::shared_ptr<const vision_msgs::msg::Detection2DArray>& yolo_msg
    );
  
};

void ImageLiDARFusion::set_param()
{
  Mat CameraExtrinsicMat_right(3, 4, CV_64F, CameraExtrinsic_vector_right.data());
  Mat CameraExtrinsicMat_left(3, 4, CV_64F, CameraExtrinsic_vector_left.data());
  Mat CameraMat_right(3, 3, CV_64F, CameraMat_vector_right.data());
  Mat CameraMat_left(3, 3, CV_64F, CameraMat_vector_left.data());
  Mat DistCoeffMat_right(1,4, CV_64F, DistCoeff_vector_right.data());
  Mat DistCoeffMat_left(1,4, CV_64F, DistCoeff_vector_left.data());

  this->transformMat_right = CameraMat_right * CameraExtrinsicMat_right;
  this->transformMat_left = CameraMat_left * CameraExtrinsicMat_left;

  cout << "transformMat_right : " << this->transformMat_right << "\n" << "transformMat_left : " << this->transformMat_left << "\n";
  RCLCPP_INFO(this->get_logger(), "parameter set end");
}

void ImageLiDARFusion::approximateSyncCallback(
    const std::shared_ptr<const vision_msgs::msg::Detection3DArray>& lidar_msg,
    const std::shared_ptr<const vision_msgs::msg::Detection2DArray>& yolo_msg)
{
  if (this->sync_locker == 0)
  {
    if(lidar_msg->detections.size() != 0)
    {
      for (int i = 0; i < lidar_msg->detections.size(); i++)
      {
        if (lidar_msg->detections[i].bbox.size.y > 0.04 && lidar_msg->detections[i].bbox.size.y < 0.4 && lidar_msg->detections[i].bbox.size.z < 1)
        {
          Box box =
          {
              lidar_msg->detections[i].bbox.center.position.x,
              lidar_msg->detections[i].bbox.center.position.y,
              lidar_msg->detections[i].bbox.center.position.z,
              lidar_msg->detections[i].bbox.size.x,
              lidar_msg->detections[i].bbox.size.y * 1.4,
              lidar_msg->detections[i].bbox.size.z * 1.6
          };
          boxes.push_back(box);
        }
      }
    }
    if(yolo_msg->detections.size() != 0)
    {
      std::string Class_name;
      for (int i = 0; i < yolo_msg->detections.size(); i++)
      {
        Class_name = yolo_msg->detections[i].results[0].hypothesis.class_id;

        int color = 0;
        if (Class_name == "blue")
        {
            color = 1;
        }
        else if (Class_name == "orange")
        {
            color = 2;
        }
        else
        {
            color = 0;
        }

        float area = yolo_msg->detections[i].bbox.size_x * yolo_msg->detections[i].bbox.size_y;
        if (area > this->min_area)
        {
          Box_yolo box_yolo =
          {
            yolo_msg->detections[i].bbox.center.position.x - ((yolo_msg->detections[i].bbox.size_x) / 2) * 1.2,
            yolo_msg->detections[i].bbox.center.position.x + ((yolo_msg->detections[i].bbox.size_x) / 2) * 1.2,
            yolo_msg->detections[i].bbox.center.position.y - (yolo_msg->detections[i].bbox.size_y) / 2,
            yolo_msg->detections[i].bbox.center.position.y + (yolo_msg->detections[i].bbox.size_y) / 2,
            color
          };
          boxes_yolo.push_back(box_yolo);
        }
      }
    }

    this->sync_locker = 1;
  }
}

void ImageLiDARFusion::FusionCallback()
{
  if (this->sync_locker == 1)
  {
    std::vector<Box> lidar_boxes = boxes;
    std::vector<Box_yolo> yolo_boxes = boxes_yolo;
    cv::Mat image(960, 640, CV_8UC3, cv::Scalar(0, 0, 0));

    if (this->yolo_locker == 0 && this->box_locker == 0)
    {
      std::vector<Box_yolo> boxes_2d_cluster;
      for (const auto &Box : lidar_boxes)
      {
        Box_points real_points = calcBox_points(Box);

        if (Box.y >= 0)
        {
          boxes_2d_cluster.push_back(transformBox(this->transformMat_left , real_points));
          cv::rectangle(image, 
            Rect(Point(boxes_2d_cluster.back().x1, boxes_2d_cluster.back().y1), 
              Point(boxes_2d_cluster.back().x2, boxes_2d_cluster.back().y2)), 
            Scalar(0, 0, 255), 2, 8, 0);
        }
        if (Box.y < 0)
        {
          boxes_2d_cluster.push_back(transformBox(this->transformMat_right , real_points ,480));
          cv::rectangle(image, 
          Rect(Point(boxes_2d_cluster.back().x1, boxes_2d_cluster.back().y1), 
            Point(boxes_2d_cluster.back().x2, boxes_2d_cluster.back().y2)), 
          Scalar(0, 0, 255), 2, 8, 0);
        }
      }

      for (const auto& Box : yolo_boxes)
      {
        if (Box.color == 1)
        {
          cv::rectangle(image, Rect(Point(Box.x1, Box.y1), Point(Box.x2, Box.y2)), Scalar(255, 0, 0), 2, 8, 0);
        }
        else if (Box.color == 2)
        {
          cv::rectangle(image, Rect(Point(Box.x1, Box.y1), Point(Box.x2, Box.y2)), Scalar(255, 0, 255), 2, 8, 0);
        }
      }

      std_msgs::msg::Float64MultiArray cone_msg;
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr coord_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (int i = 0; i < boxes_2d_cluster.size(); i++)
      {
        float max_iou = 0.0f;
        int class_id = -1;

        for (int j = 0; j < yolo_boxes.size(); j++)
        {
          float iou = get_iou(boxes_2d_cluster[i], yolo_boxes[j]);
          if (iou > max_iou)
          {
            max_iou = iou;
            class_id = yolo_boxes[j].color;
          }
        }

        if (max_iou > this->min_iou)
        {
          pcl::PointXYZRGB pointRGB;

          cv::rectangle(image, 
            Rect(Point(boxes_2d_cluster[i].x1, boxes_2d_cluster[i].y1), 
            Point(boxes_2d_cluster[i].x2, boxes_2d_cluster[i].y2)), 
            Scalar(100, 100, 0), 3, 8, 0);

          float center_x = lidar_boxes[i].x;
          float center_y = lidar_boxes[i].y;

          cone_msg.data.push_back(center_x);
          cone_msg.data.push_back(center_y);
          cone_msg.data.push_back(class_id);

          if (class_id == 1)
          {
            pointRGB.x = center_x;
            pointRGB.y = center_y;
            pointRGB.z = 0.;
            pointRGB.r = 0;
            pointRGB.g = 0;
            pointRGB.b = 255;
          }
          else if (class_id == 2)
          {
            pointRGB.x = center_x;
            pointRGB.y = center_y;
            pointRGB.z = 0.;
            pointRGB.r = 255;
            pointRGB.g = 255;
            pointRGB.b = 0;
          }
          coord_cloud->push_back(pointRGB);
        }
      }

      this->cone_pub_->publish(cone_msg);

      coord_cloud->width = 1;
      coord_cloud->height = coord_cloud->points.size();
      pcl::toROSMsg(*coord_cloud, pointcloud_msg);
      pointcloud_msg.header.frame_id = "velodyne";
      this->LiDAR_pub_->publish(pointcloud_msg);

      this->box_locker = 0;
      boxes.clear();
      this->yolo_locker = 0;
      boxes_yolo.clear();
    }
  }
  this->sync_locker = 0;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageLiDARFusion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
