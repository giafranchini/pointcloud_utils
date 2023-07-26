#ifndef PCL_UTILS_HPP_PCL_UTILS_HPP
#define PCL_UTILS_HPP_PCL_UTILS_HPP

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "pcl_ros/transforms.hpp"
#include "pointcloud_utils/tf_utils.hpp"

namespace pointcloud_utils
{
using namespace std::chrono_literals;

template <typename PointT>
void fromROSMsg(const sensor_msgs::msg::PointCloud2& cloud, pcl::PointCloud<PointT>& pcl_cloud)
{
  // copyPointCloud2MetaData
  pcl_conversions::toPCL(cloud.header, pcl_cloud.header);
  pcl_cloud.height = cloud.height;
  pcl_cloud.width = cloud.width;
  pcl_cloud.is_dense = cloud.is_dense == 1;

  // create a mapping from field name to pcl::PCLPointField
  pcl::MsgFieldMap field_map;
  std::vector<pcl::PCLPointField> pcl_msg_fields;
  pcl_conversions::toPCL(cloud.fields, pcl_msg_fields);
  pcl::createMapping<PointT>(pcl_msg_fields, field_map);

  // Copy point data
  pcl_cloud.resize(cloud.width * cloud.height);
  std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(&pcl_cloud.points[0]);
  const std::uint8_t* msg_data = &cloud.data[0];

  // Check if we can copy adjacent points in a single memcpy.  We can do so if there
  // is exactly one field to copy and it is the same size as the source and destination
  // point types.
  if (field_map.size() == 1 && field_map[0].serialized_offset == 0 && field_map[0].struct_offset == 0 &&
      field_map[0].size == cloud.point_step && field_map[0].size == sizeof(PointT))
  {
    std::uint32_t cloud_row_step = static_cast<std::uint32_t>(sizeof(PointT) * pcl_cloud.width);
    // Should usually be able to copy all rows at once
    if (cloud.row_step == cloud_row_step)
    {
      memcpy(cloud_data, msg_data, cloud.width * cloud.height * sizeof(PointT));
    }
    else
    {
      for (pcl::uindex_t i = 0; i < cloud.height; ++i, cloud_data += cloud_row_step, msg_data += cloud.row_step)
        memcpy(cloud_data, msg_data, cloud_row_step);
    }
  }
  else
  {
    // If not, memcpy each group of contiguous fields separately
    for (std::size_t row = 0; row < cloud.height; ++row)
    {
      const std::uint8_t* row_data = msg_data + row * cloud.row_step;
      for (std::size_t col = 0; col < cloud.width; ++col)
      {
        const std::uint8_t* msg_data = row_data + col * cloud.point_step;
        for (const pcl::detail::FieldMapping& mapping : field_map)
        {
          std::copy(msg_data + mapping.serialized_offset, msg_data + mapping.serialized_offset + mapping.size,
                    cloud_data + mapping.struct_offset);
        }
        cloud_data += sizeof(PointT);
      }
    }
  }
}

template <typename PointT>
void toROSMsg(const pcl::PointCloud<PointT>& pcl_cloud, sensor_msgs::msg::PointCloud2& cloud)
{
  // Ease the user's burden on specifying width/height for unorganized datasets
  if (pcl_cloud.width == 0 && pcl_cloud.height == 0)
  {
    cloud.width = pcl_cloud.size();
    cloud.height = 1;
  }
  else
  {
    assert(pcl_cloud.size() == pcl_cloud.width * pcl_cloud.height);
    cloud.height = pcl_cloud.height;
    cloud.width = pcl_cloud.width;
  }

  // Fill point cloud binary data (padding and all)
  std::size_t data_size = sizeof(PointT) * pcl_cloud.size();
  cloud.data.resize(data_size);
  if (data_size)
  {
    memcpy(&cloud.data[0], &pcl_cloud[0], data_size);
  }

  // Fill fields metadata
  std::vector<pcl::PCLPointField> pcl_msg_fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(pcl_msg_fields));
  pcl_conversions::fromPCL(pcl_msg_fields, cloud.fields);

  pcl_conversions::fromPCL(pcl_cloud.header, cloud.header);
  // cloud.header = pcl_cloud.header;
  cloud.point_step = sizeof(PointT);
  cloud.row_step = (sizeof(PointT) * cloud.width);
  cloud.is_dense = (pcl_cloud.is_dense ? 1 : 0);

  /// @todo msg.is_bigendian = ?;
}

template <typename PointT>
class GenericCondition : public pcl::ConditionBase<PointT>
{
public:
  typedef std::shared_ptr<GenericCondition<PointT>> Ptr;
  typedef std::shared_ptr<const GenericCondition<PointT>> ConstPtr;
  typedef std::function<bool(const PointT&)> FunctorT;

  GenericCondition(FunctorT evaluator) : pcl::ConditionBase<PointT>(), _evaluator(evaluator)
  {
  }

  virtual bool evaluate(const PointT& point) const
  {
    return _evaluator(point);
  }

private:
  FunctorT _evaluator;
};

}  // namespace pointcloud_utils

#endif  // PCL_UTILS_HPP_PCL_UTILS_HPP