// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <filesystem>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <iostream>
#include <memory>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>


//#include <phaser/common/point-types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


#include "open3d/Open3D.h"
#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"
#include <Eigen/Core>                   // for MatrixMap

#include <algorithm>                    // for copy_n, fill_n
#include <cstdint>                      // for uint8_t, uint32_t
#include <ostream>                      // for ostream, operator<<
#include <type_traits>                  // for enable_if_t             
// BAH, We will use the native o3d logging
//#include <glog/logging.h>
// PCL goes away
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <ros/ros.h>
DEFINE_string(
    target_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\",
    "Defines the path to the target cloud.");
DEFINE_string(
    source_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\",
    "Defines the path to the source cloud.");
DEFINE_string(
    reg_cloud, "c:\\repo\\phaser\\phaser_core\\",
    "Defines the path to the registered cloud.");

using namespace std;
using namespace open3d;
//namespace o3d = open3d;
//namespace geom = geometry;
namespace vis = visualization;


void PrintPointCloud(const geom::PointCloud& pointcloud) {
  bool pointcloud_has_normal = pointcloud.HasNormals();
  utility::LogInfo("Pointcloud has %d points.", (int)pointcloud.points_.size());

  Eigen::Vector3d min_bound = pointcloud.GetMinBound();
  Eigen::Vector3d max_bound = pointcloud.GetMaxBound();
  utility::LogInfo(
      "Bounding box is: ({:.4f}, {:.4f}, {:.4f}) - ({:.4f}, {:.4f}, "
      "{:.4f})",
      min_bound(0), min_bound(1), min_bound(2), max_bound(0), max_bound(1),
      max_bound(2));

  for (size_t i = 0; i < pointcloud.points_.size(); i++) {
    if (pointcloud_has_normal) {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      const Eigen::Vector3d& normal = pointcloud.normals_[i];
      utility::LogInfo(
          "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}", point(0), point(1),
          point(2), normal(0), normal(1), normal(2));
    } else {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      utility::LogInfo("{:.6f} {:.6f} {:.6f}", point(0), point(1), point(2));
    }
  }
  utility::LogInfo("End of the list.");
}

//  FixUpO3dColors for open3D.
//  Intensity point clouds are read into Red by o3d.
//  Current o3d (9/22) point clouds do not support seperate intensity channel.
//  For each point,
//  Set each Green Blue value  to its Red value. Also scale by maximum
//  NOTE- add a return value for convenient use in compound expressions.
//        (even though this is in-place)
geom::PointCloud& FixUpO3dColors(geom::PointCloud& pntCld) {
  double maxVal = 0.0;
  for (auto& clr : pntCld.colors_) {
    double r = clr(0);
    if (r > maxVal) maxVal = r;
  }
  if (maxVal == 0.0)
    return pntCld;
  double invMax = 1.0 / maxVal;
  for (auto& clr : pntCld.colors_) {
    double r = clr(0) * invMax;
    clr(1) = r;
    clr(2) = r;
    clr(0) = r;
  }
  return pntCld;
}
using ele = double;
union Mpoint_t {
  ele data[4];
  struct _M {
    ele x;
    ele y;
    ele z;
    ele intensity;
  } M;
};
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
/** \brief A point structure representing Euclidean xyz coordinates, and the
 * intensity value. \ingroup common
 */
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
  inline pcl::Array3fMapConst getArray3fMap() const {
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
    cout <<"x: " << p.x << " y: "<< p.y<< " z: "<<p.z<< " intensity: "<< p.intensity;
    return os;
  };
};

using Point_t = PointXYZI; 
int main(int argc, char* argv[]) {
  utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::filesystem::current_path() << std::endl;
  std::cout << FLAGS_target_cloud << " " << FLAGS_source_cloud << " "
            << FLAGS_reg_cloud << " " << std::endl;
  //  BAH, using gflags instead of o3d command line options

  vis::Visualizer vis;

  geom::PointCloud tcld, scld;
  Point_t aa(1,2,3,4);
  cout << aa<<endl;
  Mpoint_t bb = {1, 2, 3, 4};
  io::ReadPointCloudFromPLY( FLAGS_source_cloud, scld, {"XYZI", true, true, true});
  io::ReadPointCloudFromPLY( FLAGS_target_cloud, tcld, {"XYZI", true, true, true});
 

  shared_ptr<geom::PointCloud> sourceCld(&FixUpO3dColors(scld));
  shared_ptr<geom::PointCloud> targetCld(&FixUpO3dColors(tcld));

  //vis.CreateVisualizerWindow("Open3D", 1600, 900);
  //vis.AddGeometry(sourceCld);
  //vis.AddGeometry(targetCld);

  //BAH, How do we update render option
  //     in renderer?
  //vis::RenderOption().SetPointSize(1);
  //vis::RenderOption().ChangePointSize(1);
  //vis.UpdateRender();

  //vis.Run();
  //visualizer.Run();
  //visualizer.DestroyVisualizerWindow();
  double zoom =1.0/5.0;
  
  Eigen::Vector3d up = {0.0, -1.0, 0.0};
  Eigen::Vector3d look = {1.0, 1.0, 0.0};
  Eigen::Vector3d front = {0.0, 0.0, -1.0};
  
  vis::DrawGeometries( {targetCld, sourceCld}, "o3d pnt clouds for phaser", 1600, 900, 50,
      50, false, false, false, &look, &up,&front,&zoom);

  // BAH, these are next to fix up with o3d pnt cld instead of PCL
  //auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");

  //model::RegistrationResult result =ctrl->registerPointCloud(targetCld, sourceCld);

  if (!targetCld.get()->HasNormals()) {
    utility::ScopeTimer timer("Normal estimation with KNN10");
    for (int i = 0; i < 10; i++) {
      targetCld.get()->EstimateNormals(
          open3d::geometry::KDTreeSearchParamKNN(10));
    }
  }

  std::cout << targetCld.get()->normals_[0] << std::endl;
  std::cout << targetCld.get()->normals_[10] << std::endl;

  utility::ScopeTimer timer("Normal estimation with Hybrid 0.01666, 60");
  for (int i = 0; i < 20; i++) {
    targetCld.get()->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(0.01666, 60));
  }

  std::cout << targetCld.get()->normals_[0] << std::endl;
  std::cout << targetCld.get()->normals_[10] << std::endl;

  // auto downpcd = targetCld->VoxelDownSample(1.0/8.0);
  auto downpcd = targetCld;
  // 1. test basic pointcloud functions.

  geometry::PointCloud pointcloud;
  PrintPointCloud(pointcloud);

  pointcloud.points_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  pointcloud.points_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  pointcloud.points_.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  pointcloud.points_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  PrintPointCloud(pointcloud);

  // 2. test pointcloud IO.

  if (io::ReadPointCloud(FLAGS_target_cloud, pointcloud)) {
    utility::LogInfo("Successfully read {}", FLAGS_target_cloud);
  } else {
    utility::LogWarning("Failed to read {}", FLAGS_target_cloud);
  }

  // 3. test pointcloud visualization

  std::shared_ptr<geometry::PointCloud> pointcloud_ptr(
      new geometry::PointCloud);
  *pointcloud_ptr = pointcloud;
  pointcloud_ptr->NormalizeNormals();
  auto bounding_box = pointcloud_ptr->GetAxisAlignedBoundingBox();

  std::shared_ptr<geometry::PointCloud> pointcloud_transformed_ptr(
      new geometry::PointCloud);
  *pointcloud_transformed_ptr = *pointcloud_ptr;
  Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
  trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  transformation.block<3, 3>(0, 0) = static_cast<Eigen::Matrix3d>(
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()));
  pointcloud_transformed_ptr->Transform(
      trans_to_origin.inverse() * transformation * trans_to_origin);

  // 4. test operations
  *pointcloud_transformed_ptr += *pointcloud_ptr;
  visualization::DrawGeometries(
      {pointcloud_transformed_ptr}, "Combined Pointcloud");

  // 5. test downsample
  auto downsampled = pointcloud_ptr->VoxelDownSample(0.05);
  visualization::DrawGeometries({downsampled}, "Down Sampled Pointcloud");

  // 6. test normal estimation
  visualization::DrawGeometriesWithKeyCallbacks(
      {pointcloud_ptr},
      {{GLFW_KEY_SPACE,
        [&](visualization::Visualizer* vis) {
          // EstimateNormals(*pointcloud_ptr,
          //        open3d::KDTreeSearchParamKNN(20));
          pointcloud_ptr->EstimateNormals(
              open3d::geometry::KDTreeSearchParamRadius(0.05));
          utility::LogInfo("Done.");
          return true;
        }}},
      "Press Space to Estimate Normal", 1600, 900);

  // n. test end

  utility::LogInfo("End of the test.");
  return 0;
}
