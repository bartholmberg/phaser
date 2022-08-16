#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"
#include "phaser/common/core-gflags.h"
// BAH- why doesn't this work if moved to core-gflags.h

//DECLARE_string(target_cloud, "", "Defines the path to the target cloud");
//DECLARE_string(source_cloud, "", "Defines the path to the source cloud");
//DECLARE_string(reg_cloud, "", "Defines the path to the registered cloud");
// 
// 
namespace phaser_core {
DEFINE_string(target_cloud, "", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "", "Defines the path to the registered cloud.");

// BAH, TBD:set these values to good defaults,
//          similarly(not identical) named inputs _spherical_bandwidth
//          in phaser core lib source, why?
DEFINE_int32(phaser_core_spherical_bandwidth, 150, "spherical bandwidth");
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

static model::PointCloudPtr readPointCloud(const std::string& path_to_ply) {
  CHECK(!path_to_ply.empty());

  //std::string tmp ="C:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\target_1.ply";

  //tmp ="C:\\repo\\phaser\\phaser_core\\RelWithDebInfo\\target1.ply";
  auto tmp2 = path_to_ply;
  //LOG(INFO) << "Reading point cloud from " << tmp;
  std::cout << "Reading point cloud from " << path_to_ply;
  model::PointCloudPtr cloud = std::make_shared<model::PointCloud>(tmp2);
  return cloud;
}

static void writePointCloud(
    const std::string& reg_file, model::PointCloudPtr cloud) {
  CHECK(!reg_file.empty());
  CHECK_NOTNULL(cloud);
  pcl::io::savePLYFileASCII(reg_file, *cloud->getRawCloud());
  LOG(INFO) << "Wrote registered cloud to " << reg_file;
}

static void registerCloud(
    const std::string& target, const std::string& source,
    const std::string& reg_cloud) {
  model::PointCloudPtr target_cloud = readPointCloud(target);
  model::PointCloudPtr source_cloud = readPointCloud(source);
  CHECK_NOTNULL(target_cloud);
  CHECK_NOTNULL(source_cloud);
  auto tptr = target_cloud->getRawCloud();
  std::cout << "\n register cloud" <<std ::endl;
  CHECK(!reg_cloud.empty());

  auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");
  model::RegistrationResult result =
      ctrl->registerPointCloud(target_cloud, source_cloud);

  LOG(INFO) << "Registration result: " << result.getStateAsVec().transpose();
  writePointCloud(reg_cloud, result.getRegisteredCloud());
}

}  // namespace phaser_core
   // namespace phaser_core
int main(int argc, char** argv) {
  ros::init(argc, argv, "phaser_core_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  VLOG(1) << "=== PHASER CORE DRIVER =====================";
  phaser_core::registerCloud(
      phaser_core::FLAGS_target_cloud, phaser_core::FLAGS_source_cloud,
      phaser_core::FLAGS_reg_cloud);

  return 0;
}