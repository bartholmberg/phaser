#ifndef PHASER_COMMON_POINT_TYPES_H_
#define PHASER_COMMON_POINT_TYPES_H_

#include <Eigen/Dense>

#include <ostream>
#include <filesystem>
#include <gflags/gflags.h>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

//#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
namespace o3d = open3d;
namespace geom = open3d::geometry;
namespace common {

using Point_t = pcl::PointXYZI;
using PointCloud_t = geom::PointCloud;
using PointCloud_tPtr = std::shared_ptr<geom::PointCloud>;
//using Ptr = shared_ptr<PointCloud<PointT> >;
//using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;
using Vector_t = Eigen::Vector3d;
}  // namespace common

#endif  // PHASER_COMMON_POINT_TYPES_H_
