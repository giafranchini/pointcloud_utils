#include "pointcloud_processing/pointcloud_transform.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <regex>
#include <chrono>

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

namespace pointcloud_processing
{


PointCloudTransform::PointCloudTransform(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_transform", options)
{
  subscription_pcl_ = this->create_subscription<Adapter>(
      "pointcloud_in", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudTransform::transform_callback, this, std::placeholders::_1));

  publisher_pcl_ = this->create_publisher<Adapter>("pointcloud_out", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribed to pointclouds");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  declare_parameter("in_frame", "map");
  declare_parameter("out_frame", "base_link");
  declare_parameter("is_static_tf", true);
  declare_parameter("transform_tolerance", 0.5);
  
  is_static_tf_ = get_parameter("is_static_tf").as_bool();
  in_frame_id_ = get_parameter("in_frame").as_string();
  out_frame_id_ = get_parameter("out_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  
  
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

void PointCloudTransform::transform_callback(const StampedPointCloud_PCL::ConstSharedPtr msg)
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  StampedPointCloud_PCL::UniquePtr transformed_pcl(new StampedPointCloud_PCL);
  // transformed_pcl->cloud.emplace<msg->cloud.index>()
  std::visit([&dest_cloud = transformed_pcl->cloud](auto&& cloud) {
        dest_cloud = decltype(dest_cloud)(cloud);
    }, msg->cloud);
  if (!is_static_tf_)
  {
    geometry_msgs::msg::TransformStamped in_to_out_msg;
    try
    {
      in_to_out_msg = tf_buffer_->lookupTransform(out_frame_id_.c_str(), in_frame_id_.c_str(), msg->header.stamp);
      pointcloud_utils::transformAsMatrix(in_to_out_msg, in_to_out_mat_);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", in_frame_id_.c_str(), out_frame_id_.c_str(),
                   ex.what());
      return;
    }
  }
  if (in_frame_id_ != out_frame_id_)
  {
    std::visit(
      [&](auto && in_cloud, auto && transformed_cloud) {
      using T = typename std::decay_t<decltype(in_cloud)>::PointType;
      using U = typename std::decay_t<decltype(transformed_cloud)>::PointType;
      if constexpr (std::is_same_v<T, U>)
      {
        RCLCPP_DEBUG(this->get_logger(), "transforming pointcloud from %s to %s: point type %s IS THE SAME",
                     in_frame_id_.c_str(), out_frame_id_.c_str(), typeid(T).name());
        if constexpr(std::is_same_v<T, pcl::PointNormal> || std::is_same_v<T, pcl::PointXYZRGBNormal> || std::is_same_v<T, pcl::PointXYZINormal> || std::is_same_v<T, pcl::PointXYZLNormal>)
        {
          RCLCPP_DEBUG(this->get_logger(), "transforming pointcloud from %s to %s: point type %s IS NORMAL",
                     in_frame_id_.c_str(), out_frame_id_.c_str(), typeid(T).name());
          pcl::transformPointCloudWithNormals(in_cloud, transformed_cloud, in_to_out_mat_);
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "transforming pointcloud from %s to %s: point type %s IS NOT NORMAL",
                     in_frame_id_.c_str(), out_frame_id_.c_str(), typeid(T).name());
          pcl::transformPointCloud(in_cloud, transformed_cloud, in_to_out_mat_);
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "transforming pointcloud from %s to %s: point type %s and %s ARE NOT THE SAME",
                     in_frame_id_.c_str(), out_frame_id_.c_str(), typeid(T).name(), typeid(U).name());
      }
      }
    , msg->cloud, transformed_pcl->cloud);
  }
  else
  {
    publisher_pcl_->publish(std::move(*msg));
  }
  transformed_pcl->header = msg->header;
  transformed_pcl->header.frame_id = out_frame_id_.c_str();
  RCLCPP_DEBUG(this->get_logger(), "header stamp: %f \t frame_id: %s",
              transformed_pcl->header.stamp.sec + transformed_pcl->header.stamp.nanosec * 1e-9, transformed_pcl->header.frame_id.c_str());
  publisher_pcl_->publish(std::move(transformed_pcl));
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(
    get_logger(), "Time difference = %ld [ms]",
    std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

}  // namespace pointcloud_processing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_processing::PointCloudTransform)

