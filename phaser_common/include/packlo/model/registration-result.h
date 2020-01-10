#ifndef PACKLO_MODEL_REGISTRATION_RESULT_H_
#define PACKLO_MODEL_REGISTRATION_RESULT_H_

#include "packlo/distribution/base-distribution.h"
#include "packlo/model/point-cloud.h"
#include "packlo/model/state.h"

#include <array>
#include <memory>

namespace model {

class RegistrationResult {
 public:
  RegistrationResult();
  explicit RegistrationResult(
      model::PointCloud&& reg_cloud, std::array<double, 3>&& rotation);
  explicit RegistrationResult(
      model::PointCloud&& reg_cloud, common::Vector_t&& rotation);
  explicit RegistrationResult(model::PointCloudPtr reg_cloud);

  RegistrationResult combine(RegistrationResult&& other);

  model::PointCloudPtr getRegisteredCloud() const;
  void setRegisteredCloud(model::PointCloudPtr reg_cloud);

  std::array<double, 3> getRotation() const;
  const common::Vector_t& getTranslation() const;

  bool foundSolution() const;
  bool foundSolutionForRotation() const;
  bool foundSolutionForTranslation() const;

  void setRotUncertaintyEstimate(
      const common::BaseDistributionPtr& uncertainty);
  void setPosUncertaintyEstimate(
      const common::BaseDistributionPtr& uncertainty);
  common::BaseDistributionPtr getRotUncertaintyEstimate() const noexcept;
  common::BaseDistributionPtr getPosUncertaintyEstimate() const noexcept;

 private:
  model::PointCloudPtr reg_cloud_;
  std::array<double, 3> rotation_;
  common::Vector_t translation_;
  bool found_solution_for_rotation_;
  bool found_solution_for_translation_;
  common::BaseDistributionPtr uncertainty_;
  State current_state_;
};

}  // namespace model

#endif  // PACKLO_MODEL_REGISTRATION_RESULT_H_
