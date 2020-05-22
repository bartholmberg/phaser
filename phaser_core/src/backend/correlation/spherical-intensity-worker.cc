#include "phaser/backend/correlation/spherical-intensity-worker.h"

#include <glog/logging.h>

#include "phaser/backend/correlation/spherical-correlation-low-pass.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/core-gflags.h"

namespace correlation {

SphericalIntensityWorker::SphericalIntensityWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values)
    : f_values_(f_values), h_values_(h_values) {
  sph_corr_.reset(new SphericalCorrelation(
      common::FLAGS_phaser_core_spherical_bandwidth,
      common::FLAGS_phaser_core_spherical_zero_padding));
}

void SphericalIntensityWorker::run() {
  CHECK_NOTNULL(sph_corr_);
  VLOG(1) << "[SphericalIntensityWorker] Estimating rotation...";

  std::vector<double> f_intensities;
  std::vector<double> h_intensities;
  std::function<double(const model::FunctionValue&)> func =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func, &f_intensities);
  convertFunctionValues(h_values_, func, &h_intensities);
  sph_corr_->correlateSampledSignals(f_intensities, h_intensities);

  is_completed_ = true;
}

std::vector<double> SphericalIntensityWorker::getCorrelation() const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return sph_corr_->getCorrelation();
}

const SphericalCorrelation& SphericalIntensityWorker::getCorrelationObject()
    const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return *sph_corr_;
}

}  // namespace correlation
