#include "phaser/backend/registration/sph-opt-registration.h"

#include <algorithm>
#include <fftw3.h>
#include <glog/logging.h>
#include <iostream>

#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/correlation/spherical-combined-worker.h"
#include "phaser/backend/correlation/spherical-intensity-worker.h"
#include "phaser/backend/correlation/spherical-range-worker.h"
#include "phaser/backend/uncertainty/bingham-peak-based-eval.h"
#include "phaser/backend/uncertainty/gaussian-peak-based-eval.h"
#include "phaser/common/core-gflags.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/statistic-utils.h"
#include "phaser/common/translation-utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
namespace phaser_core {

SphOptRegistration::SphOptRegistration()
    : BaseRegistration("SphOptRegistration"),
      bandwidth_(FLAGS_phaser_core_spherical_bandwidth),
      sampler_(FLAGS_phaser_core_spherical_bandwidth) {
  BaseEvalPtr rot_eval = std::make_unique<BinghamPeakBasedEval>();
  BaseEvalPtr pos_eval = std::make_unique<GaussianPeakBasedEval>();
  correlation_eval_ = std::make_unique<PhaseCorrelationEval>(
      std::move(rot_eval), std::move(pos_eval));
  auto a = fftw_init_threads();
  //CHECK_NE(fftw_init_threads(), 0);
  fftw_plan_with_nthreads(12);
}
SphOptRegistration::~SphOptRegistration() {}

model::RegistrationResult SphOptRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK(cloud_prev);
  CHECK(cloud_cur);
  std::cout<< "=== Registering point cloud ==="<< std::endl;
  std::cout << "Cloud1: " << cloud_prev->getPlyReadDirectory() << std::endl;
  std::cout << "Cloud2: " << cloud_cur->getPlyReadDirectory() << std::endl;
  cloud_prev->initialize_kd_tree();

  // Register the point cloud.
  model::RegistrationResult result = estimateRotation(cloud_prev, cloud_cur);

  //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  // BAH, viewer doesn't like raw point cloud object???
  //viewer.showCloud(*cloud_cur->getRawCloud());
  //  BAH, save out rot only pnt cld
  //pcl::io::savePLYFileBinary("rotonly.ply", *cloud_cur->getRawCloud());
  estimateTranslation(cloud_prev, &result);
  return result;
}

model::RegistrationResult SphOptRegistration::estimateRotation(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  std::cout << "[SphOptRegistration] Estimating rotation..." << std::endl;
  // Correlate point cloud and get uncertainty measure.
  std::vector<SphericalCorrelation>* correlations =
      correlatePointcloud(cloud_prev, cloud_cur);
  auto corrTemp = *correlations;
  //SphericalCorrelation& corr = correlations[0];
  SphericalCorrelation& corr = corrTemp[0];

  common::BaseDistributionPtr rot =
      correlation_eval_->calcRotationUncertainty(corr);
  Eigen::Vector4d inv = rot->getEstimate();
  inv.block(1, 0, 3, 1) = -inv.block(1, 0, 3, 1);
  Eigen::VectorXd b_est =
      common::RotationUtils::ConvertQuaternionToXYZ(rot->getEstimate());

  std::cout << "\nBingham q: " << rot->getEstimate().transpose() << std::endl;
  std::cout  << "\nBingham rotation: " << b_est.transpose()*180.0/M_PI << std::endl;
  //BAH, use -rot(y) because y axis is pointed down for K4a
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, b_est(0), b_est(1), b_est(2));

  model::RegistrationResult result(std::move(*cloud_cur));
  result.setRotUncertaintyEstimate(rot);
  result.setRotationCorrelation(corr.getCorrelation());

  return result;
}

void SphOptRegistration::estimateTranslation(
    model::PointCloudPtr cloud_prev, model::RegistrationResult* result) {
  std::cout << "[SphOptRegistration] Estimating translation..." << std::endl;

  model::PointCloudPtr rot_cloud = result->getRegisteredCloud();
  const double duration_translation_f_ms = common::executeTimedFunction(
      &phaser_core::BaseAligner::alignRegistered, &aligner_, *cloud_prev,
      f_values_, *rot_cloud, h_values_);
  common::BaseDistributionPtr pos =
      correlation_eval_->calcTranslationUncertainty(aligner_);
  Eigen::VectorXd g_est = pos->getEstimate();

  std::cout << "Gaussian translation: " << g_est.transpose() << std::endl;
  std::cout << "Translational alignment took: " << duration_translation_f_ms << "ms." << std::endl;
  

  common::TranslationUtils::TranslateXYZ(
      rot_cloud, g_est(0), g_est(1), g_est(2));
  result->setPosUncertaintyEstimate(pos);
}

void SphOptRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  BaseRegistration::getStatistics(manager);
}
static int tcnt = 0;
using std::vector;
vector<SphericalCorrelation> *SphOptRegistration::correlatePointcloud(
    model::PointCloudPtr target, model::PointCloudPtr source) {
  source->initialize_kd_tree();
  target->initialize_kd_tree();
  std::cout << "SphOptRegistration::correlatePointcloud" << std::endl;
  // Sample the sphere at the grid points.
  vector<model::FunctionValue>* f_values =new vector<model::FunctionValue>;
  vector<model::FunctionValue>* h_values  = new vector < model::FunctionValue>;

  std::cout << " sample uniformly" << std::endl;
  // BAH: optionally save out clouds projected onto sphere
  //
  //"targetPrj.ply"
  sampler_.sampleUniformly(*target, f_values );
  sampler_.sampleUniformly(*source, h_values);
  // Create workers for the spherical correlation.
  // SphericalIntensityWorkerPtr corr_intensity_worker = CHECK_NOTNULL(
  // std::make_shared<SphericalIntensityWorker>(f_values, h_values));
  // SphericalRangeWorkerPtr corr_range_worker =
  // CHECK_NOTNULL(std::make_shared<SphericalRangeWorker>(f_values, h_values));
  SphericalCombinedWorkerPtr corr_combined_worker = CHECK_NOTNULL(
      std::make_shared<SphericalCombinedWorker>(*f_values, *h_values));

  // Add workers to pool and execute them.
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "--------------- add worker thread: corr_combined_worker. Count: "<< tcnt++ << std::endl;
  th_pool_.add_worker(corr_combined_worker);
  th_pool_.run_and_wait_all();
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "\nTime for rot est: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count()
          << "ms\n";
  
  int* b = new int(1);
  vector<SphericalCorrelation>* tmpOut =new std::vector<SphericalCorrelation>{corr_combined_worker->getCorrelationObject()};
  //vector<SphericalCorrelation> tmpOut {corr_combined_worker->getCorrelationObject()};
  corr_combined_worker->shutdown();
  return tmpOut;
}

void SphOptRegistration::setBandwith(const int bandwith) {
  sampler_.initialize(bandwith);
}

BaseEval& SphOptRegistration::getRotEvaluation() {
  CHECK_NOTNULL(correlation_eval_);
  return correlation_eval_->getRotationEval();
}

BaseEval& SphOptRegistration::getPosEvaluation() {
  CHECK_NOTNULL(correlation_eval_);
  return correlation_eval_->getPositionEval();
}

}  // namespace phaser_core
