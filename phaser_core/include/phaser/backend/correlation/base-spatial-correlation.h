#ifndef PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace correlation {

class BaseSpatialCorrelation {
 public:
  virtual double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) = 0;
};

using BaseSpatialCorrelationPtr = std::unique_ptr<BaseSpatialCorrelation>;

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
