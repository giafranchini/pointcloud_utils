#ifndef POINTCLOUD_UTILS__POINT_TYPES_CUSTOM_HPP_ 
#define POINTCLOUD_UTILS__POINT_TYPES_CUSTOM_HPP_
 
#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Include the PCL headers for the algorithms that we want to use with custom point types
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/registration/incremental_registration.h>
#include <pcl/registration/impl/incremental_registration.hpp>

#include <ostream>

namespace pcl
{
  struct PointXYZIRT;
/** \brief A point structure representing Euclidean xyz coordinates the intensity value, the ring and the time value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 _PointXYZIRT
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float intensity;
      };
      float data_c[4];
    };
    std::uint16_t ring;
    float time;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZIRT& p);
  struct PointXYZIRT : public _PointXYZIRT
  {
    inline constexpr PointXYZIRT (const _PointXYZIRT &p) : PointXYZIRT{p.x, p.y, p.z, p.intensity, p.ring, p.time} {}

    inline constexpr PointXYZIRT (float _intensity = 0.f, std::uint16_t _ring = 0, float _time = 0.f) : PointXYZIRT(0.f, 0.f, 0.f, _intensity, _ring, _time) {}

    inline constexpr PointXYZIRT (float _x, float _y, float _z, float _intensity = 0.f, std::uint16_t _ring = 0, float _time = 0.f) : 
      _PointXYZIRT{{{_x, _y, _z, 1.0f}}, {{_intensity}}, _ring, _time } {}
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZIRT& p);
  };

  struct PointXYZT;
  /** \brief A point structure representing Euclidean xyz coordinates and the temperature value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 _PointXYZT    // enforce SSE padding for correct memory alignment
  {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float temperature;
    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZT& p);
  struct PointXYZT : public _PointXYZT
  {
    inline constexpr PointXYZT (const _PointXYZT &p) : PointXYZT{p.x, p.y, p.z, p.temperature} {}

    inline constexpr PointXYZT (float _temperature = 0.f) : PointXYZT(0.f, 0.f, 0.f, _temperature) {}

    inline constexpr PointXYZT (float _x, float _y, float _z, float _temperature = 0.f) : 
      _PointXYZT{{{_x, _y, _z}}, {_temperature}} {}
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZT& p);
  };

  // Define a new point representation for < x, y, z, temperature >
  class PointXYZTRepresentation : public PointRepresentation <PointXYZT>
  {
    using PointRepresentation<PointXYZT>::nr_dimensions_;

  public:
    PointXYZTRepresentation ()
    {
      // Define the number of dimensions
      nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointXYZT &p, float * out) const
    {
      // < x, y, z, temperature >
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.temperature;
    }
  };

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZIRT,           // here we assume a XYZ + "test" (as fields)
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZT,           // here we assume a XYZ + "temperature" (as fields)
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, temperature, temperature)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIRT, pcl::_PointXYZIRT)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZT, pcl::_PointXYZT)

#endif // POINTCLOUD_UTILS__POINT_TYPES_CUSTOM_HPP_



