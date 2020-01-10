#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/common/data/datasource-ply.h"
#include "packlo/common/metric-utils.h"
#include "packlo/common/test/testing-entrypoint.h"
#include "packlo/common/test/testing-predicates.h"
#include "packlo/distribution/gaussian.h"

#include <gtest/gtest.h>

namespace uncertainty {

class TransGaussTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/arche/");
    registrator_ = std::make_unique<registration::SphRegistration>(
        "phase", "bingham", "gaussian");
    z_score_eval_ = dynamic_cast<correlation::ZScoreEval*>(
        &registrator_->getPosEvaluation());
  }

  data::DatasourcePlyPtr ds_;
  registration::SphRegistrationPtr registrator_;
  correlation::ZScoreEval* z_score_eval_;
};

TEST_F(TransGaussTest, LowUncertainty) {
  CHECK(ds_);
  z_score_eval_->getPeakExtraction().getScoreThreshold() = 5.45;

  model::RegistrationResult result;
  model::PointCloudPtr prev_cloud = nullptr;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    VLOG(1) << "------------------------";
    CHECK(cloud);
    if (prev_cloud == nullptr) {
      prev_cloud = cloud;
      prev_cloud->initialize_kd_tree();
      return;
    }

    // Register the point clouds.
    cloud->initialize_kd_tree();
    result = registrator_->registerPointCloud(prev_cloud, cloud);
    VLOG(1) << "------------------------";
    prev_cloud = cloud;
    EXPECT_TRUE(result.foundSolutionForRotation());
    EXPECT_TRUE(result.foundSolutionForTranslation());
    VLOG(1) << "------------------------";

    auto rot = result.getRotUncertaintyEstimate();
    CHECK_NOTNULL(rot);
    auto pos = result.getPosUncertaintyEstimate();
    CHECK_NOTNULL(pos);
    common::GaussianPtr uncertainty =
        std::dynamic_pointer_cast<common::Gaussian>(
            result.getPosUncertaintyEstimate());
    VLOG(1) << "cov------------------------";
    CHECK_NOTNULL(uncertainty);
    const Eigen::MatrixXd& cov = uncertainty->getCov();
    EXPECT_LT(cov.trace(), 15);
    VLOG(1) << "cov2------------------------";
  });
  ds_->startStreaming(0);
}

}  // namespace uncertainty

MAPLAB_UNITTEST_ENTRYPOINT
