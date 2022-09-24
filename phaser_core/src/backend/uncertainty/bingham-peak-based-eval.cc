#include "phaser/backend/uncertainty/bingham-peak-based-eval.h"

#include <algorithm>
//#include <glog/logging.h>

#include "phaser/common/rotation-utils.h"



namespace phaser_core {
DEFINE_int32(bingham_peak_neighbors, 0,"Determines the number of neighbors used for the Bingham calculation.");


common::BaseDistributionPtr BinghamPeakBasedEval::evaluatePeakBasedCorrelation(
    const uint32_t bw, const std::set<uint32_t>& signals,
    const std::vector<double>& normalized_corr) const {
  common::BinghamPtr bingham = std::make_shared<common::Bingham>(
      fitRotationalBinghamDistribution(bw, signals, normalized_corr));
  return bingham;
}

common::Bingham BinghamPeakBasedEval::fitRotationalBinghamDistribution(
    const uint32_t bw, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  const uint32_t n_signals = signals.size();
  const uint32_t n_corr = norm_corr.size();
  //CHECK_NE(n_signals, 0);
  std::cout << "Checking " << n_signals << " signals for evaluation";
  std::set<uint32_t>::iterator max_signal = std::max_element(
      signals.begin(), signals.end(),
      [&norm_corr](const uint32_t lhs, const uint32_t rhs) {
        return norm_corr[lhs] < norm_corr[rhs];
      });
  //CHECK(max_signal != signals.end());
  //  BAH, weight length == end-start + 1 
  uint32_t start, end;
// allocate data into A_vec

 
  calculateStartEndNeighbor(*max_signal, n_corr, &start, &end);
  // BAH, start/end are the same in the example, which does not seem
  // correct (for weight normalization). So set to 'reasonable value' until
  // can figure out why
  end = end + 10; 
  start = start - 10;
  std::vector<double> viewW(end - start + 1);  
  //Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(4, 2 * FLAGS_bingham_peak_neighbors + 1);
  //Eigen::RowVectorXd weights = Eigen::RowVectorXd::Zero(2 * FLAGS_bingham_peak_neighbors + 1);
  Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(4, viewW.size());
  Eigen::RowVectorXd weights = Eigen::RowVectorXd::Zero(viewW.size());
  //Eigen::Map<Eigen::RowVectorXd> weights(viewW.data(), viewW.size(), 1);

  retrievePeakNeighbors(bw, start, end, norm_corr, &samples, &weights);
  // BAH, view weights (in debugger)
  viewW.insert( viewW.end(), std::make_move_iterator(weights.data()), std::make_move_iterator(weights.data() + weights.size()));

  return common::Bingham::fit(samples, weights);
}

void BinghamPeakBasedEval::calculateStartEndNeighbor(
    const uint32_t index, const uint32_t n_corr, uint32_t* start,
    uint32_t* end) const {
  //CHECK_NOTNULL(start);
  //CHECK_NOTNULL(end);
  *start = FLAGS_bingham_peak_neighbors > index
               ? index
               : index - FLAGS_bingham_peak_neighbors;
  *end = FLAGS_bingham_peak_neighbors + index > n_corr
             ? index
             : index + FLAGS_bingham_peak_neighbors;
}

void BinghamPeakBasedEval::retrievePeakNeighbors(
    const uint32_t bw, const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr, Eigen::MatrixXd* samples,
    Eigen::RowVectorXd* weights) const {
  //CHECK_NOTNULL(samples);
  //CHECK_NOTNULL(weights);
  const uint32_t n_signals = norm_corr.size();

  //CHECK_GT(n_signals, 0u);
  //CHECK_GE(start, 0u);
  //CHECK_LT(end, n_signals);
  //CHECK_LE(start, end);

  std::cout << "Checking bingham neighbors from " << start << " to " << end;
  std::size_t k = 0u;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<double, 3> zyz = common::RotationUtils::GetZYZFromIndex(i, bw);
    Eigen::Quaterniond q = common::RotationUtils::ConvertZYZtoQuaternion(zyz);
    (*samples)(0, k) = q.w();
    (*samples)(1, k) = q.x();
    (*samples)(2, k) = q.y();
    (*samples)(3, k) = q.z();
    (*weights)(k) = norm_corr.at(i);
    ++k;
  }
  // BAH, we are getting k==1 in test case
  // so weights has only one value, and 
  if (k == 1) {
    std::cout << "\n warning only one value for weight in this case, so normalization makes no sense" << std::endl;
    return;
  }
  const double weight_sum = weights->array().sum();
  //CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;
}

}  // namespace phaser_core
