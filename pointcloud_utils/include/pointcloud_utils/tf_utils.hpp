#ifndef POINTCLOUD_UTILS_TF_UTILS_HPP_
#define POINTCLOUD_UTILS_TF_UTILS_HPP_


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <Eigen/Core>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pointcloud_utils
{

bool transformPoseInTargetFrame(const geometry_msgs::msg::PoseStamped& input_pose,
                                geometry_msgs::msg::PoseStamped& transformed_pose,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string target_frame,
                                const double transform_timeout);
bool getCurrentPose(geometry_msgs::msg::PoseStamped& global_pose, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                    const std::string global_frame, const std::string robot_frame, const double transform_timeout,
                    const rclcpp::Time stamp);

bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                  const tf2::Duration& transform_tolerance, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                  geometry_msgs::msg::TransformStamped& transform);

/** \brief Obtain the transformation matrix from TF into an Eigen form
 * \param bt the TF transformation
 * \param out_mat the Eigen transformation
 */
void transformAsMatrix(const tf2::Transform& bt, Eigen::Matrix4f& out_mat);

/** \brief Obtain the transformation matrix from TF into an Eigen form
 * \param bt the TF transformation
 * \param out_mat the Eigen transformation
 */
void transformAsMatrix(const geometry_msgs::msg::TransformStamped& bt, Eigen::Matrix4f& out_mat);
}  // namespace pointcloud_utils

#endif /* POINTCLOUD_UTILS_TF_UTILS_HPP_ */
