#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/common/statistics-manager.h"

#include <memory>
#include <vector>

namespace alignment {

class BaseAligner {
  public:
    virtual void alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg,
      common::Vector_t* xyz) = 0;

    virtual std::vector<double> getCorrelation() const = 0;
};

using BaseAlignerPtr = std::unique_ptr<BaseAligner>;

} // namespace alignment
