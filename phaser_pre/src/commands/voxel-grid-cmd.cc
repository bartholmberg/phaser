#include "phaser_pre/commands/voxel-grid-cmd.h"

//#include <glog/logging.h>

DEFINE_double(
    phaser_voxel_grid_leaf_size, 0.25f,
    "Defines the leaf size of the voxel grid.");

namespace preproc {

void VoxelGridCmd::execute(model::PointCloudPtr cloud) {
  //CHECK_NOTNULL(cloud);
  std::cout << "[PreProcessing] Performing voxel grid filtering..." <<std::endl;
  // BAH, Having troubline linking these get**Cloud() routines
  common::PointCloud_tPtr input_cloud;//= cloud->getRawCloud();
  //CHECK_NOTNULL(input_cloud);
  //BAH, comment out for now
  /*
  voxel_grid_filter_.setInputCloud(input_cloud);
  voxel_grid_filter_.setLeafSize(
      FLAGS_phaser_voxel_grid_leaf_size, FLAGS_phaser_voxel_grid_leaf_size,
      FLAGS_phaser_voxel_grid_leaf_size);
  voxel_grid_filter_.filter(*input_cloud);
  */
  std::cout << "[PreProcessing] Check implementation" << std::endl;
}

}  // namespace preproc
