#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
//#include <ros/ros.h>
#include <filesystem>
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
DEFINE_string(target_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "c:\\repo\\phaser\\phaser_test_data\\test_clouds\\os0\\", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "c:\\repo\\phaser\\phaser_core\\", "Defines the path to the registered cloud.");

// BAH, TBD:set these values to good defaults,
//          similarly(not identical) named inputs _spherical_bandwidth
//          in phaser core lib source, why?
DEFINE_int32(phaser_core_spherical_bandwidth, 150, "spherical bandwidth"); //150 original
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


void MakeKinectDat(std::string const& inPcdName, std::string const& outPlyName) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudout(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB>(inPcdName, *cloudin);  //* load the file
  cloudout->points.resize(cloudin->size());
  int k = 0;
  for (size_t i = 0; i < cloudin->points.size(); i++) {
    float iVal =
        cloudin->points[i].r + cloudin->points[i].g + cloudin->points[i].b;
    if (iVal > 0) {
      cloudout->points[k].x = 10 * cloudin->points[i].x / 1000.0;
      cloudout->points[k].y = 10 * cloudin->points[i].y / 1000.0;
      cloudout->points[k].z = 10 * cloudin->points[i].z / 1000.0;
      cloudout->points[k].intensity = iVal * iVal;
      k++;
    }
  };
  cloudout->points.resize(k);
  pcl::io::savePLYFileASCII(outPlyName, *cloudout);

}
int main(int argc, char** argv) {
  //ros::init(argc, argv, "phaser_core_driver");

  //MakeKinectDat( "C:\\repo\\bart\\demo\\room3\\pcd_0006.pcd", phaser_core::FLAGS_source_cloud + "source_4.ply");
  //MakeKinectDat("C:\\repo\\bart\\demo\\room3\\pcd_0014.pcd", phaser_core::FLAGS_target_cloud + "target_4.ply");

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  std::cout << std::filesystem::current_path() << std::endl;

  std::cout << "=== PHASER CORE DRIVER =====================" << std::endl;
  phaser_core::registerCloud(
      phaser_core::FLAGS_target_cloud, phaser_core::FLAGS_source_cloud,
      phaser_core::FLAGS_reg_cloud);

  return 0;
}