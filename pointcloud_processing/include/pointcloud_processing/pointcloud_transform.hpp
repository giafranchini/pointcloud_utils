#ifndef POINTCLOUD_PROCESSING_POINTCLOUD_TRANSFORM_HPP_
#define POINTCLOUD_PROCESSING_POINTCLOUD_TRANSFORM_HPP_

#include "pointcloud_utils/pcl_utils.hpp"
#include "pointcloud_utils/tf_utils.hpp"
#include "native_adapters/PCL.hpp"

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
class PointCloudTransform : public rclcpp::Node
{
public:
  explicit PointCloudTransform(const rclcpp::NodeOptions& options);

private:
  using Adapter = rclcpp::adapt_type<StampedPointCloud_PCL>::as<sensor_msgs::msg::PointCloud2>;
  void transform_callback(const StampedPointCloud_PCL::ConstSharedPtr msg);

  rclcpp::Subscription<Adapter>::SharedPtr subscription_pcl_;
  rclcpp::Publisher<Adapter>::SharedPtr publisher_pcl_;
  std::string in_frame_id_, out_frame_id_;

  
  // tf buffer to get transfroms
  double transform_tolerance_;
  bool is_static_tf_;
  
  Eigen::Matrix4f in_to_out_mat_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pointcloud_processing

#endif  // POINTCLOUD_PROCESSING_POINTCLOUD_TRANSFORM_HPP_
