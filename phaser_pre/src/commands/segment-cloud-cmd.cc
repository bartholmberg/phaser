#include "phaser_pre/commands/segment-cloud-cmd.h"

//#include <glog/logging.h>

namespace preproc {

void SegmentCloudCmd::execute(model::PointCloudPtr cloud) {
  std::cout << "[PreProcessing] Performing geometric segmentation..." <<std::endl;
  const ProjectionResult proj_result = proj_.projectPointCloud(cloud);
  const ClusterResult cluster_result =
      cluster_.cluster(proj_result.getRangeMat(), proj_result.getSignalMat());
  const GroundRemovalResult ground_result =
      gnd_removal_.removeGround(proj_result.getFullCloud());
  const SegmentationResult seg_result =
      seg_.segment(proj_result, cluster_result, ground_result);

  //BAH, Having troubline linking these get**Cloud() routines
  //cloud->getRawCloud();//= seg_result.getSegmentedCloud();
  //cloud->getRawInfoCloud();//= seg_result.getSegmentedInfoCloud();
}

}  // namespace preproc
