#include "pointcloud_utils/point_types_custom.hpp"

#include <pcl/common/common.h>
#include <pcl/point_types.h>

namespace pcl
{
  std::ostream& operator << (std::ostream& os, const PointXYZIRT& p)
  {
    os << "(" << p.x << ", " << p.y << ", " << p.z << " - " << p.intensity << ", " << p.ring << ", " << p.time << ")";
    return (os);
  }

  std::ostream& operator << (std::ostream& os, const PointXYZT& p)
  {
    os << "(" << p.x << ", " << p.y << ", " << p.z << " - " << p.temperature << ")";
    return (os);
  }
}
