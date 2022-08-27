#pragma once

//#include <boost/function.hpp>
#include <memory>

#include "phaser/model/point-cloud.h"

namespace data {

class BaseDatasource {
 public:
  virtual void subscribeToPointClouds(
      std::function<void(const model::PointCloudPtr&)> func) = 0;
  virtual void startStreaming(const uint32_t number_of_clouds = 0) = 0;

 protected:
  std::vector<std::function<void(const model::PointCloudPtr&)>> callbacks_;
};

using DatasourcePtr = std::shared_ptr<BaseDatasource>;

}  // namespace data
