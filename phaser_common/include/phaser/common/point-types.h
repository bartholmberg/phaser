#ifndef PHASER_COMMON_POINT_TYPES_H_
#define PHASER_COMMON_POINT_TYPES_H_

#include <Eigen/Dense>

#include <ostream>
#include <filesystem>
#include <gflags/gflags.h>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

namespace o3d = open3d;
namespace geom = open3d::geometry;
namespace common {

using Array3fMap = Eigen::Map<Eigen::Array3f>;
using Array3fMapConst = const Eigen::Map<const Eigen::Array3f>;
using Array4fMap = Eigen::Map<Eigen::Array4f, Eigen::Aligned>;
using Array4fMapConst = const Eigen::Map<const Eigen::Array4f, Eigen::Aligned>;
using Vector3fMap = Eigen::Map<Eigen::Vector3f>;
using Vector3fMapConst = const Eigen::Map<const Eigen::Vector3f>;
using Vector4fMap = Eigen::Map<Eigen::Vector4f, Eigen::Aligned>;
using Vector4fMapConst =
    const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned>;

using Vector3c = Eigen::Matrix<std::uint8_t, 3, 1>;
using Vector3cMap = Eigen::Map<Vector3c>;
using Vector3cMapConst = const Eigen::Map<const Vector3c>;
using Vector4c = Eigen::Matrix<std::uint8_t, 4, 1>;
using Vector4cMap = Eigen::Map<Vector4c, Eigen::Aligned>;
using Vector4cMapConst = const Eigen::Map<const Vector4c, Eigen::Aligned>;
// This adds the members x,y,z which can also be accessed using the point
// (which is float[4])
struct alignas(16) _PointXYZI {
  union alignas(16) {
    float data[4];
    struct {
      float x;
      float y;
      float z;
    };
  };
  inline Vector3fMap getVector3fMap() {
    return (Vector3fMap(data));
  }
  inline Vector3fMapConst getVector3fMap() const {
    return (Vector3fMapConst(data));
  }
  inline Vector4fMap getVector4fMap() {
    return (Vector4fMap(data));
  }
  inline Vector4fMapConst getVector4fMap() const {
    return (Vector4fMapConst(data));
  }
  inline Array3fMap getArray3fMap() {
    return (Array3fMap(data));
  }
  inline Array3fMapConst getArray3fMap() const {
    return (pcl::Array3fMapConst(data));
  }
  inline Array4fMap getArray4fMap() {
    return (Array4fMap(data));
  }
  inline Array4fMapConst getArray4fMap() const {
    return (Array4fMapConst(data));
  }
  union {
    struct {
      float intensity;
    };
    float data_c[4];
  };
  using _custom_allocator_type_trait = void;
};

//std::ostream& operator<<(std::ostream& os, const PointXYZI& p);

struct PointXYZI : public _PointXYZI {
  inline PointXYZI(const _PointXYZI& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    intensity = p.intensity;
  }

  inline PointXYZI(float _intensity = 0.f)
      : PointXYZI(0.f, 0.f, 0.f, _intensity) {}

  inline PointXYZI(float _x, float _y, float _z, float _intensity = 0.f) {
    x = _x;
    y = _y;
    z = _z;
    data[3] = 1.0f;
    intensity = _intensity;
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZI& p) {
    std::cout <<"x: " << p.x << " y: "<< p.y<< " z: "<<p.z<< " intensity: "<< p.intensity;
    return os;
  };
};

//BAH, need  to recreate Point_t_old for Point_t , 
// but without PCL

//using Point_t_old = pcl::PointXYZI;
using Point_t = pcl::PointXYZI; //swap in working
//using Point_t_new = std::vector<Eigen::Vector3d>;
using PointCloud_t = geom::PointCloud;
using PointCloud_tPtr = std::shared_ptr<geom::PointCloud>;
//using Ptr = shared_ptr<PointCloud<PointT> >;
//using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;
using Vector_t = Eigen::Vector3d;
}  // namespace common

#endif  // PHASER_COMMON_POINT_TYPES_H_
