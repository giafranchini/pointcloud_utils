#include "pointcloud_utils/tf_utils.hpp"

namespace pointcloud_utils
{

bool transformPoseInTargetFrame(const geometry_msgs::msg::PoseStamped& input_pose,
                                geometry_msgs::msg::PoseStamped& transformed_pose,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string target_frame,
                                const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try
  {
    transformed_pose = tf_buffer->transform(input_pose, target_frame, tf2::durationFromSec(transform_timeout));
    return true;
  }
  catch (tf2::LookupException& ex)
  {
    RCLCPP_ERROR(logger, "No Transform available Error looking up target frame: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(logger, "Connectivity Error looking up target frame: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up target frame: %s\n", ex.what());
  }
  catch (tf2::TimeoutException& ex)
  {
    RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", transform_timeout);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to transform from %s to %s", input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}

bool getCurrentPose(geometry_msgs::msg::PoseStamped& global_pose, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                    const std::string global_frame, const std::string robot_frame, const double transform_timeout,
                    const rclcpp::Time stamp)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  return transformPoseInTargetFrame(global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}

bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                  const tf2::Duration& transform_tolerance, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                  geometry_msgs::msg::TransformStamped& transform)
{
  // tf2_transform.setIdentity();  // initialize by identical transform

  if (source_frame_id == target_frame_id)
  {
    // We are already in required frame
    return true;
  }

  try
  {
    // Obtaining the transform to get data from source to target frame
    transform = tf_buffer->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero, transform_tolerance);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("getTransform"), "Failed to get \"%s\"->\"%s\" frame transform: %s",
                 source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }
  return true;
}

void transformAsMatrix(const tf2::Transform& bt, Eigen::Matrix4f& out_mat)
{
  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);

  tf2::Vector3 origin = bt.getOrigin();

  out_mat(0, 0) = mv[0];
  out_mat(0, 1) = mv[4];
  out_mat(0, 2) = mv[8];
  out_mat(1, 0) = mv[1];
  out_mat(1, 1) = mv[5];
  out_mat(1, 2) = mv[9];
  out_mat(2, 0) = mv[2];
  out_mat(2, 1) = mv[6];
  out_mat(2, 2) = mv[10];

  out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
  out_mat(3, 3) = 1;
  out_mat(0, 3) = origin.x();
  out_mat(1, 3) = origin.y();
  out_mat(2, 3) = origin.z();
}

void transformAsMatrix(const geometry_msgs::msg::TransformStamped& bt, Eigen::Matrix4f& out_mat)
{
  tf2::Transform transform;
  tf2::convert(bt.transform, transform);
  transformAsMatrix(transform, out_mat);
}

}  // namespace pointcloud_utils
