#include "OptionsTLIO.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsTLIO::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser == nullptr)
    return;

  std::string f = "config_tlio";

  if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
    enabled = false;
    return;
  }
  parser->parse_external(f, "tlio", "enabled", enabled);
  parser->parse_external(f, "tlio", "topic", topic);
  parser->parse_external(f, "tlio", "timeoffset", dt);
  parser->parse_external(f, "tlio", "noise_o", noise_o);
  parser->parse_external(f, "tlio", "noise_p", noise_p);
  parser->parse_external(f, "tlio", "chi2_mult", chi2_mult);
  parser->parse_external(f, "tlio", "do_calib_dt", do_calib_dt);
  parser->parse_external(f, "tlio", "do_calib_ext", do_calib_ext);
  parser->parse_external(f, "tlio", "init_cov_dt", init_cov_dt);
  parser->parse_external(f, "tlio", "init_cov_ex_o", init_cov_ex_o);
  parser->parse_external(f, "tlio", "init_cov_ex_p", init_cov_ex_p);
}

void mins::OptionsTLIO::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - Camera\n" RESET);
  PRINT1("\t\t- topic: %s\n", topic.c_str());
}
