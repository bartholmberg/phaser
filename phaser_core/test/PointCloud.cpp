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
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/utility/Optional.h"

#include <phaser/common/point-types.h>
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
namespace phaser_core {

DEFINE_string(
    target_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\",
    "Defines the path to the target cloud.");
DEFINE_string(
    source_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\",
    "Defines the path to the source cloud.");
DEFINE_string(
    reg_cloud, "c:\\repo\\phaser\\phaser_core\\",
    "Defines the path to the registered cloud.");

// BAH, TBD:set these values to good defaults,
//          similarly(not identical) named inputs _spherical_bandwidth
//          in phaser core lib source, why?
DEFINE_int32(
    phaser_core_spherical_bandwidth, 150,
    "spherical bandwidth");  // 150 original
DEFINE_int32(phaser_core_spherical_zero_padding, 10, "zero pad");
DEFINE_int32(
    phaser_core_spherical_low_pass_lower_bound, 0, "low pass - lower band");
DEFINE_int32(
    phaser_core_spherical_low_pass_upper_bound, 10000, "low pass - upper band");

DEFINE_int32(phaser_core_spatial_n_voxels, 201, "");
DEFINE_int32(phaser_core_spatial_discretize_lower, -50, "");
DEFINE_int32(phaser_core_spatial_discretize_upper, 50, "");
DEFINE_int32(phaser_core_spatial_zero_padding, 0, "");
DEFINE_int32(phaser_core_spatial_low_pass_lower_bound, 85, "");
DEFINE_int32(phaser_core_spatial_low_pass_upper_bound, 115, "");
// namespace phaser_core
}  // namespace phaser_core

using namespace std;
using namespace open3d;
using namespace Eigen;
//namespace o3d = open3d;
//namespace geom = geometry;
namespace vis = visualization;
namespace cor = phaser_core;

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
  
  double Scale = 1.0;
  for (auto& clr : pntCld.colors_) {
    double r = clr(0) * Scale;
    clr(1) = r;
    clr(2) = r;
    clr(0) = r;
  }
  return pntCld;
}


 

 // MakeModelCloud. Input name of ply cloud.
 // Get 'raw' open3d cloud to construct a PHASER model pnt
 // cloud
 model::PointCloudPtr MakeModelCloud(const std::string & fN, double voxelSize=0.05) {

   geom::PointCloud* gcld= new geom::PointCloud();
   gcld->SetName(fN);
   //io::ReadPointCloudFromPCD(fN, *gcld, {"XYZI", true, true, true});
   io::ReadPointCloudFromPLY(fN, *gcld, {"XYZI", true, true, true});
  
   common::PointCloud_tPtr pntCldPntr(&FixUpO3dColors(*gcld));
   pntCldPntr=pntCldPntr->VoxelDownSample(voxelSize);
   model::PointCloud* mCld = new model::PointCloud(pntCldPntr);

 
   model::PointCloudPtr mCldPtr(mCld);
   if (!mCldPtr->getRawCloud()->HasNormals()) {
     utility::ScopeTimer timer("Normal estimation with KNN10");
     for (int i = 0; i < 10; i++) {
       mCldPtr->getRawCloud()->EstimateNormals(
           geom::KDTreeSearchParamKNN(10));
       mCldPtr->getRawCloud()->EstimateNormals(
           geom::KDTreeSearchParamKNN(10));
     }
   }
   mCldPtr->setPlyReadDirectory(fN);
   return mCldPtr;
 }
 
int main(int argc, char* argv[]) {
  utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << std::filesystem::current_path() << std::endl;
  std::cout << cor::FLAGS_target_cloud << " " << cor::FLAGS_source_cloud << " "
            << cor::FLAGS_reg_cloud << " " << std::endl;
  //  BAH, using gflags instead of o3d command line options
  //  BAH, do we need to remove glog from phaser???
  vis::Visualizer vis;

  //geom::PointCloud tcld, scld;
  common::Point_t aa(1,2,3,4);
  cout << aa<<endl;
 

  model::PointCloudPtr sourceCld = MakeModelCloud(cor::FLAGS_source_cloud,0.25);
  model::PointCloudPtr targetCld = MakeModelCloud(cor::FLAGS_target_cloud,0.25);

  std::cout << sourceCld->getRawCloud()->GetName() << " " << std::endl;
  double zoom =1.0/5.0;
  
  Eigen::Vector3d up = {0.0, -1.0, 0.0};
  Eigen::Vector3d look = {1.0, 1.0, 0.0};
  Eigen::Vector3d front = {0.0, 0.0, -1.0};

  common::PointCloud_tPtr fooSrc((sourceCld->clone()).getRawCloud());
  fooSrc->PaintUniformColor({0.0,0.4,0.5});
  vis::DrawGeometries(
      {fooSrc,
       targetCld->getRawCloudScaledColor()}, 
      "o3d pnt clouds for phaser", 1600, 900, 50,
      50, false, false, false, &look, &up,&front,&zoom);

  
  auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");
  


  model::RegistrationResult result =
      ctrl->registerPointCloud(targetCld, sourceCld);

 
  return 0;
  std::cout << targetCld->getRawCloud()->normals_[0] << std::endl;
  std::cout << sourceCld->getRawCloud()->normals_[10] << std::endl;

  utility::ScopeTimer timer("Normal estimation with Hybrid 0.01666, 60");
  for (int i = 0; i < 20; i++) {
    targetCld->getRawCloud()->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(0.01666, 60));
  }

  std::cout << targetCld->getRawCloud()->normals_[0] << std::endl;
  std::cout << targetCld->getRawCloud()->normals_[10] << std::endl;
  // simple nearest neighbor search example
  geometry::KDTreeFlann fooKd(*sourceCld->getRawCloud());
  int nn = std::min(20, (int)sourceCld->getRawCloud()->points_.size() - 1);
  std::vector<int> iVec(nn);
  std::vector<double> dVec(nn);
  // BAH, simple example of o3d nearest neighbor search
  fooKd.SearchKNN(sourceCld->getRawCloud()->points_[0], nn, iVec, dVec);
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

  if (io::ReadPointCloud(cor::FLAGS_target_cloud, pointcloud)) {
    utility::LogInfo("Successfully read {}", cor::FLAGS_target_cloud);
  } else {
    utility::LogWarning("Failed to read {}", cor::FLAGS_target_cloud);
  }

  // 3. test pointcloud visualization

  std::shared_ptr<geom::PointCloud> pointcloud_ptr( new geom::PointCloud);
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
//}  // namespace phaser_core


  // vis.CreateVisualizerWindow("Open3D", 1600, 900);
// vis.AddGeometry(sourceCld);
// vis.AddGeometry(targetCld);

// BAH, How do we update render option
//      in renderer?
// vis::RenderOption().SetPointSize(1);
// vis::RenderOption().ChangePointSize(1);
// vis.UpdateRender();

// vis.Run();
// visualizer.Run();
// visualizer.DestroyVisualizerWindow();