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

/*
// Points coordinates.
    std::vector<Eigen::Vector3d> points_;
    /// Points normals.
    std::vector<Eigen::Vector3d> normals_;
    /// RGB colors of points.
    std::vector<Eigen::Vector3d> colors_;
    /// Covariance Matrix for each point
    std::vector<Eigen::Matrix3d> covariances_;
 */
/*
* C:\repo\vcpkg\installed\x64-windows\include\pcl\impl\point_types.hpp
*  struct EIGEN_ALIGN16 _PointXYZI
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
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  struct PointXYZI : public _PointXYZI
  {
    inline PointXYZI (const _PointXYZI &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      intensity = p.intensity;
    }

    inline PointXYZI (float _intensity = 0.f): PointXYZI(0.f, 0.f, 0.f, _intensity) {}

    inline PointXYZI (float _x, float _y, float _z, float _intensity = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      intensity = _intensity;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  };

*/ 

//BAH, need  to recreate Point_t_old for Point_t , 
// but without PCL

using Point_t_old = pcl::PointXYZI;
using Point_t = pcl::PointXYZI; //swap in working
using Point_t_new = std::vector<Eigen::Vector3d>;
using PointCloud_t = geom::PointCloud;
using PointCloud_tPtr = std::shared_ptr<geom::PointCloud>;
//using Ptr = shared_ptr<PointCloud<PointT> >;
//using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;
using Vector_t = Eigen::Vector3d;
}  // namespace common

#endif  // PHASER_COMMON_POINT_TYPES_H_
