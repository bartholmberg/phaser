#include "phaser/common/spherical-sampler.h"
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <cmath>
//#include <glog/logging.h>

namespace common {

SphericalSampler::SphericalSampler(const int bandwith)
    : is_initialized_(false) {
  initialize(bandwith);
}

void SphericalSampler::initialize(const int bandwith) {
  const std::vector<common::Point_t> sample_angles = create2BwGrid(bandwith);
  cartesian_grid_ = convertCartesian(sample_angles);
  is_initialized_ = true;
  bandwith_ = bandwith;
}

  model::PointCloud SphericalSampler::sampleUniformly(
    const model::PointCloud& cloud, std::vector<model::FunctionValue>* grid) {
  static int cnt = 0;
  //CHECK(is_initialized_);
  grid->clear();
  //model::PointCloud& sphere = projection_.convertPointCloudCopy(cloud);
  common::PointCloud_tPtr fooSphere(projection_.convertPointCloudCopy(cloud).getRawCloud());
  model::PointCloud* sphere = new model::PointCloud(fooSphere);
  //model::PointCloud* sphereP =new model::PointCloud (projection_.convertPointCloudCopy(cloud));
  // BAH, save out spherical projected cloud, 
  //      Note,this function gets scheduled as a Task (in places), and any change to interface
  //      breaks the task (why?).  


  sphere->initialize_kd_tree();
  sphere->getNearestPoints(cartesian_grid_, grid);
  return *sphere;
  }

std::vector<common::Point_t> SphericalSampler::create2BwGrid(
    const std::size_t bw) {
  std::cout << "bw arg for grid: " << bw << std::endl;
  std::vector<common::Point_t> sample_angles;
  const std::size_t grid = 2 * bw - 1;
  for (std::size_t i = 0u; i <= grid; ++i) {
    const float x = (M_PI * (2 * i + 1)) / (4 * bw);
    for (std::size_t j = 0u; j <= grid; ++j) {
      common::Point_t p;
      p.x = x;
      p.y = (2 * M_PI * j) / (2 * bw);
      sample_angles.emplace_back(std::move(p));
    }
  }
  std::cout << "Created spherical DH-Grid with " << sample_angles.size()
          << " samples." << std::endl;

  return sample_angles;
}

std::vector<common::Point_t> SphericalSampler::convertCartesian(
    const std::vector<common::Point_t>& grid) {
  std::vector<common::Point_t> res;
  const float n_grid = static_cast<float>(grid.size()) / 25;
  const float step_distance = 0.00;
  std::cout << "step distance = " << step_distance << " n: " << n_grid
            << std::endl;
  float dist = 1.0f;
  for (const common::Point_t& p : grid) {
    common::Point_t cart_p;
    cart_p.x = dist * std::sin(p.x) * std::cos(p.y);
    cart_p.y = dist * std::sin(p.x) * std::sin(p.y);
    cart_p.z = dist * std::cos(p.x);
    res.emplace_back(std::move(cart_p));
    dist += step_distance;
  }
  return res;
}

int SphericalSampler::getInitializedBandwith() const noexcept {
  //CHECK(is_initialized_);
  return bandwith_;
}

}  // namespace common
