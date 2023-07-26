#include "pointcloud_processing/pointcloud_merge.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <regex>

namespace pointcloud_processing
{

PointCloudMerge::PointCloudMerge(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_merge", options)
{
  subscription_left_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera_left/pointcloud_out", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudMerge::left_callback, this, std::placeholders::_1));

  subscription_right_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera_right/pointcloud_out", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudMerge::right_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_merged", 10);
  merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_merged_tf", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribed to pointclouds");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  merged_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  left_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  right_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  conditional_filtered_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  transformed_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  obstacle_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  declare_parameter("in_frame", "map");
  declare_parameter("out_frame", "base_link");
  declare_parameter("upper_limit", 2.0);
  declare_parameter("lower_limit", 0.0);
  declare_parameter("width_pcl", 0.5);
  declare_parameter("is_static_tf", true);
  declare_parameter("transform_tolerance", 0.5);
  declare_parameter("door_voxel_size_xy", 0.03f);
  declare_parameter("door_voxel_size_z", 0.03f);
  declare_parameter("obstacle_voxel_size_xy", 0.06f);
  declare_parameter("obstacle_voxel_size_z", 0.02f);

  is_static_tf_ = get_parameter("is_static_tf").as_bool();
  in_frame_id_ = get_parameter("in_frame").as_string();
  out_frame_id_ = get_parameter("out_frame").as_string();
  upper_limit_ = get_parameter("upper_limit").as_double();
  lower_limit_ = get_parameter("lower_limit").as_double();
  width_pcl_ = get_parameter("width_pcl").as_double();
  door_voxel_size_xy_ = get_parameter("door_voxel_size_xy").as_double();
  door_voxel_size_z_ = get_parameter("door_voxel_size_z").as_double();
  obstacle_voxel_size_xy_ = get_parameter("obstacle_voxel_size_xy").as_double();
  obstacle_voxel_size_z_ = get_parameter("obstacle_voxel_size_z").as_double();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  conditional_filter_ = std::make_shared<pcl::ConditionalRemoval<pcl::PointXYZ>>();
  obstacle_filter_ = std::make_shared<pcl::ConditionalRemoval<pcl::PointXYZ>>();
  voxel_filter_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
  voxel_filter_obstacle_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
  configure_filters();
  callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ =
      this->create_wall_timer(500ms, std::bind(&PointCloudMerge::publish_cloud_callback, this), callback_group_timer_);
  // timer_ = this->create_wall_timer(500ms, std::bind(&PointCloudMerge::publish_cloud_callback, this));

  // Create the filtering object
  if (is_static_tf_)
  {
    while (!tf_buffer_->canTransform(in_frame_id_, out_frame_id_, tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for transform between %s and %s", in_frame_id_.c_str(),
                  out_frame_id_.c_str());
    }
    geometry_msgs::msg::TransformStamped in_to_out_msg;
    pointcloud_utils::getTransform(in_frame_id_, out_frame_id_, tf2::durationFromSec(10 * transform_tolerance_),
                                 tf_buffer_, in_to_out_msg);
    pointcloud_utils::transformAsMatrix(in_to_out_msg, in_to_out_mat_);
    RCLCPP_INFO(this->get_logger(), "Got transform between %s and %s", in_frame_id_.c_str(), out_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Transform: %f %f %f", in_to_out_msg.transform.translation.x,
                in_to_out_msg.transform.translation.y, in_to_out_msg.transform.translation.z);
  }
  RCLCPP_INFO(this->get_logger(), "initialization finished");
}

void PointCloudMerge::configure_filters()
{
  conditional_filter_->setInputCloud(merged_pcl_);
  obstacle_filter_->setInputCloud(merged_pcl_);
  std::regex camera_regex("optical");
  if (std::regex_search(in_frame_id_, camera_regex))
  {
    RCLCPP_INFO(this->get_logger(), "Frame contains optical, we are using y to filter");
    // condition filter for door
    auto range_cond =
        std::make_shared<pointcloud_utils::GenericCondition<pcl::PointXYZ>>([=](const pcl::PointXYZ& point) {
          return point.y > -upper_limit_ && point.y < -lower_limit_ && point.x > -width_pcl_ / 2 &&
                 point.x < width_pcl_ / 2;
        });
    conditional_filter_->setCondition(range_cond);

    // condition filter for obstacle
    auto obstacle_cond =
        std::make_shared<pointcloud_utils::GenericCondition<pcl::PointXYZ>>([=](const pcl::PointXYZ& point) {
          return point.y > -lower_limit_ && point.x > -width_pcl_ / 2 && point.x < width_pcl_ / 2;
        });
    obstacle_filter_->setCondition(obstacle_cond);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Frame not contains optical, we are using z to filter");
    // condition filter for door
    auto range_cond =
        std::make_shared<pointcloud_utils::GenericCondition<pcl::PointXYZ>>([=](const pcl::PointXYZ& point) {
          return point.z > lower_limit_ && point.z < upper_limit_ && point.x > -width_pcl_ / 2 &&
                 point.x < width_pcl_ / 2;
        });
    conditional_filter_->setCondition(range_cond);

    // condition filter for obstacle
    auto obstacle_cond =
        std::make_shared<pointcloud_utils::GenericCondition<pcl::PointXYZ>>([=](const pcl::PointXYZ& point) {
          return point.z < lower_limit_ && point.x > -width_pcl_ / 2 && point.x < width_pcl_ / 2;
        });
  }
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  voxel_filter_->setInputCloud(conditional_filtered_pcl_);
  voxel_filter_->setLeafSize(door_voxel_size_xy_, door_voxel_size_xy_, door_voxel_size_z_);
  voxel_filter_obstacle_->setInputCloud(obstacle_pcl_);
  voxel_filter_obstacle_->setLeafSize(obstacle_voxel_size_xy_, obstacle_voxel_size_xy_, obstacle_voxel_size_z_);
}

void PointCloudMerge::right_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(this->get_logger(), "right callback");
  last_header_ = msg->header;
  pointcloud_utils::fromROSMsg(*msg, *right_pcl_);
  *merged_pcl_ = *right_pcl_;
  *merged_pcl_ += *left_pcl_;
}

void PointCloudMerge::left_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(this->get_logger(), "left callback");
  last_header_ = msg->header;
  pointcloud_utils::fromROSMsg(*msg, *left_pcl_);
  *merged_pcl_ = *left_pcl_;
  *merged_pcl_ += *right_pcl_;
}

void PointCloudMerge::publish_cloud_callback()
{
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    RCLCPP_DEBUG(this->get_logger(), "publish callback");

    if (merged_pcl_->empty())
    {
      RCLCPP_DEBUG(this->get_logger(), "empty merged cloud");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "merged: %ld", merged_pcl_->size());

    // myMultiplePassThrough(merged_pcl_, filtered_pcl_, filter_fields_, filter_limits_)
    conditional_filter_->filter(*conditional_filtered_pcl_);
    obstacle_filter_->filter(*obstacle_pcl_);
    if (conditional_filtered_pcl_->empty())
    {
      RCLCPP_DEBUG(this->get_logger(), "empty conditional_filtered_pcl_");
      return;
    }
    voxel_filter_->filter(*filtered_pcl_);
    RCLCPP_DEBUG(this->get_logger(), "filtered voxel: %ld", filtered_pcl_->size());

    if (!obstacle_pcl_->empty())
    {
      // RCLCPP_DEBUG(this->get_logger(), "empty obstacle_pcl_");
      voxel_filter_obstacle_->filter(*obstacle_pcl_);
      RCLCPP_DEBUG(this->get_logger(), "filtered voxel obstacle: %ld", obstacle_pcl_->size());
    }
  }
  auto out_msg = sensor_msgs::msg::PointCloud2();
  RCLCPP_DEBUG(this->get_logger(), "transform init");
  if (!is_static_tf_)
  {
    geometry_msgs::msg::TransformStamped in_to_out_msg;
    try
    {
      in_to_out_msg = tf_buffer_->lookupTransform(out_frame_id_.c_str(), in_frame_id_.c_str(), out_msg.header.stamp);
      pointcloud_utils::transformAsMatrix(in_to_out_msg, in_to_out_mat_);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", in_frame_id_.c_str(), out_frame_id_.c_str(),
                   ex.what());
      return;
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "transform fin1");
  if (in_frame_id_ != out_frame_id_)
  {
    RCLCPP_DEBUG(this->get_logger(), "transform fin2");  // TODO use PCL transform and Eigen
    pcl::transformPointCloud(*filtered_pcl_, *transformed_pcl_, in_to_out_mat_);
    RCLCPP_DEBUG(this->get_logger(), "transform fin3");
    pcl::transformPointCloud(*obstacle_pcl_, *obstacle_pcl_, in_to_out_mat_);
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "transform fin4");
    transformed_pcl_.swap(filtered_pcl_);
  }
  RCLCPP_DEBUG(this->get_logger(), "transform fin");
  // door pcl
  pointcloud_utils::toROSMsg(*transformed_pcl_, out_msg);
  out_msg.header = last_header_;
  out_msg.header.frame_id = out_frame_id_.c_str();
  RCLCPP_DEBUG(this->get_logger(), "header stamp: %f \t frame_id: %s",
               out_msg.header.stamp.sec + out_msg.header.stamp.nanosec * 1e-9, out_msg.header.frame_id.c_str());
  publisher_->publish(out_msg);

  // obstacle pcl
  pointcloud_utils::toROSMsg(*obstacle_pcl_, out_msg);
  out_msg.header = last_header_;
  out_msg.header.frame_id = out_frame_id_.c_str();
  RCLCPP_DEBUG(this->get_logger(), "header stamp: %f \t frame_id: %s",
               out_msg.header.stamp.sec + out_msg.header.stamp.nanosec * 1e-9, out_msg.header.frame_id.c_str());
  merged_publisher_->publish(out_msg);

  RCLCPP_DEBUG(this->get_logger(), "published");
}

}  // namespace pointcloud_processing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_processing::PointCloudMerge)
