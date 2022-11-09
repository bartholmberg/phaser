#include "phaser_pre/commands/pass-through-gnd-filter-cmd.h"

//#include <glog/logging.h>

DEFINE_double(
    phaser_pass_through_gnd_filter_limit_min, -5.0,
    "Defines the lower bound of the pass through GND filter.");
DEFINE_double(
    phaser_pass_through_gnd_filter_limit_max, 3.0,
    "Defines the upper bound of the pass through GND filter.");

namespace preproc {

void PassThroughGndFilterCmd::execute(model::PointCloudPtr cloud) {
  std::cout  << "[PreProcessing] Performing pass through GND filtering..." <<std::endl;
  // BAH, Having troubline linking these get**Cloud() routines
  // BAH, comment out for now
  common::PointCloud_tPtr input_cloud; //= cloud->getRawCloud();
  // BAH, comment out for now
  /*
  gnd_filter_.setInputCloud(input_cloud);
  gnd_filter_.setFilterFieldName("z");
  gnd_filter_.setFilterLimits(
      FLAGS_phaser_pass_through_gnd_filter_limit_min,
      FLAGS_phaser_pass_through_gnd_filter_limit_max);
  gnd_filter_.setFilterLimitsNegative(true);
  gnd_filter_.filter(*input_cloud);
  */
}

}  // namespace preproc
