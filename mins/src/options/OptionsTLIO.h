#ifndef MINS_OPTIONSTLIO_H
#define MINS_OPTIONSTLIO_H

#include <memory>
#include <string>
#include <Eigen/Eigen>

namespace ov_core {
class YamlParser;
struct FeatureInitializerOptions;
} // namespace ov_core

namespace mins {
struct OptionsTLIO {
  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);
  void print();

  bool enabled = false;

  double noise_o = 0.01;
  double noise_p = 0.01;
  
  double chi2_mult = 1;

  double init_cov_dt = 1e-3;
  double init_cov_ex_o = 1e-3;
  double init_cov_ex_p = 1e-3;

  double dt;

  Eigen::VectorXd extrinsics;

  bool do_calib_dt = false;
  bool do_calib_ext = false;

  std::string topic = "/tlio";
};
} // namespace mins

#endif