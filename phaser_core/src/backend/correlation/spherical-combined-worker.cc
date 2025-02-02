﻿#include "phaser/backend/correlation/spherical-combined-worker.h"

#include <glog/logging.h>

#include "phaser/backend/correlation/spherical-correlation-laplace.h"
#include "phaser/common/core-gflags.h"

namespace phaser_core {

SphericalCombinedWorker::SphericalCombinedWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values)
    : f_values_(f_values), h_values_(h_values) {

    auto bwTmp = FLAGS_phaser_core_spherical_bandwidth;
    auto zpTmp = FLAGS_phaser_core_spherical_zero_padding;
    auto sclTmp = new SphericalCorrelationLaplace(bwTmp, zpTmp);
    sph_corr_.reset(sclTmp);
}

void SphericalCombinedWorker::run() {
  CHECK_NOTNULL(sph_corr_);
  std::cout << "[SphericalCombinedWorker::run ] Estimating rotation ..." << std::endl;
  

  // Get the intensities.
  SampledSignal f_intensities;
  SampledSignal h_intensities;
  std::function<double(const model::FunctionValue&)> func_intensities =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func_intensities, &f_intensities);
  convertFunctionValues(h_values_, func_intensities, &h_intensities);

  // Get the ranges.
  SampledSignal f_range;
  SampledSignal h_range;
  std::function<double(const model::FunctionValue&)> func_range =
      [](const model::FunctionValue& v) { return v.getAveragedRange(); };
  convertFunctionValues(f_values_, func_range, &f_range);
  convertFunctionValues(h_values_, func_range, &h_range);

  // Get the reflectivities.
  //SampledSignal f_reflectivity;
  //SampledSignal h_reflectivity;
  //std::function<double(const model::FunctionValue&)> func_reflectivity =
  //    [](const model::FunctionValue& v) { return v.getAveragedReflectivity(); };
  //convertFunctionValues(f_values_, func_reflectivity, &f_reflectivity);
  //convertFunctionValues(h_values_, func_reflectivity, &h_reflectivity);

  // Get the ambient points.
  //SampledSignal f_ambient;
  //SampledSignal h_ambient;
 // std::function<double(const model::FunctionValue&)> func_ambient =
 //     [](const model::FunctionValue& v) { return v.getAveragedAmbientNoise(); };
 // convertFunctionValues(f_values_, func_ambient, &f_ambient);
 // convertFunctionValues(h_values_, func_ambient, &h_ambient);
  //BAH, f/g memory freed when the thread completes. Like all 
  //     memory on the stack
  //std::vector<SampledSignal> f={f_range, f_intensities, f_reflectivity, f_ambient},g={h_range, h_intensities, h_reflectivity, h_ambient};
  std::vector<SampledSignal> f = {f_range, f_intensities}, g = {h_range, h_intensities};
  sph_corr_->correlateSampledSignals(f,g);
  is_completed_ = true;
}

std::vector<double> SphericalCombinedWorker::getCorrelation() const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return sph_corr_->getCorrelation();
}

const SphericalCorrelation& SphericalCombinedWorker::getCorrelationObject()
    const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return *sph_corr_;
}

void SphericalCombinedWorker::shutdown() {
  CHECK_NOTNULL(sph_corr_);
  sph_corr_->shutdown();
}

}  // namespace phaser_core
