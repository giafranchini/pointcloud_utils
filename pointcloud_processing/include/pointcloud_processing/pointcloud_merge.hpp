#ifndef POINTCLOUD_PROCESSING_POINTCLOUD_MERGE_HPP_
#define POINTCLOUD_PROCESSING_POINTCLOUD_MERGE_HPP_

#include "pointcloud_utils/pcl_utils.hpp"
#include "pointcloud_utils/tf_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/utils.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <regex>

#include <rclcpp/rclcpp.hpp>

namespace pointcloud_processing
{
using namespace std::chrono_literals;
class PointCloudMerge : public rclcpp::Node
{
public:
  explicit PointCloudMerge(const rclcpp::NodeOptions& options);

private:
  void configure_filters();
  int filter_cloud();
  void left_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void right_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_cloud_callback();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_pcl_, subscription_left_pcl_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, merged_publisher_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl_, transformed_pcl_, left_pcl_, right_pcl_, filtered_pcl_,
      conditional_filtered_pcl_, obstacle_pcl_;
  // pcl::PassThrough<pcl::PointXYZ>::Ptr pass_filter_height_, pass_filter_width_;
  std::shared_ptr<pcl::ConditionalRemoval<pcl::PointXYZ>> conditional_filter_, obstacle_filter_;
  pcl::VoxelGrid<pcl::PointXYZ>::Ptr voxel_filter_, voxel_filter_obstacle_;
  std_msgs::msg::Header last_header_;
  std::string in_frame_id_, out_frame_id_;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
  double upper_limit_, lower_limit_, width_pcl_, transform_tolerance_;
  double door_voxel_size_xy_, door_voxel_size_z_, obstacle_voxel_size_xy_, obstacle_voxel_size_z_;

  // tf buffer to get transfroms
  bool is_static_tf_;
  // geometry_msgs::msg::TransformStamped in_to_out_msg_;
  Eigen::Matrix4f in_to_out_mat_;
  std::recursive_mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pointcloud_processing
#endif /* POINTCLOUD_PROCESSING_POINTCLOUD_MERGE_HPP_ */
