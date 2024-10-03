/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "functions/ResultTrajectory.h"
#if ROS_AVAILABLE == 2
#include "rclcpp/rclcpp.hpp"
#else
#include "ros/ros.h"
#endif
#include "utils/Loader.h"
#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace std;
using namespace ov_eval;
using namespace boost;
using namespace Eigen;
typedef boost::filesystem::path PATH;
typedef vector<PATH> VEC_PATH;
// file and directory paths
string align_mode, folder_groundtruths, folder_algorithms;
VEC_PATH path_gts, path_als;
// struct to record statistics
map<string, map<PATH, pair<Statistics, Statistics>>> ATE, NEES;
map<string, map<PATH, map<double, pair<Statistics, Statistics>>>> RPE;
map<string, map<double, pair<Statistics, Statistics>>> RPE_ALL;
map<string, map<PATH, Statistics>> TIME;
map<PATH, double> GT_DIST;
// Final visualization type
int viz_type = 0;
// max column for ate table
int max_col = 11;
// RPE segment lengths
vector<double> segments = {10, 20, 50};

/// Read the files and setup for trajectory error evaluation
void basic_setup(int argc, char **argv);

/// Merge two vectors to the left one
void insert_to_left(std::vector<double> &a, const std::vector<double> &b) { a.insert(a.end(), b.begin(), b.end()); }

/// Get paths and files of the ground truths and estimated trajectory
VEC_PATH get_path_groundtruths();
VEC_PATH get_path_algorithms();
void get_paths_of_run_files(map<string, PATH> datasets, const PATH &path_gt, vector<string> &run_paths, vector<string> &time_paths);
void get_timing(string time_path, Statistics &time_);
map<string, PATH> get_list_of_datasets(const PATH &path);
map<PATH, pair<double, double>> get_best_ATE(VEC_PATH v_gt, map<string, map<PATH, pair<Statistics, Statistics>>> ATE);

/// Initialize record struc
map<string, map<PATH, pair<Statistics, Statistics>>> ATE_init();
map<string, map<PATH, pair<Statistics, Statistics>>> NEES_init();
map<PATH, double> GT_DIST_init();
map<string, map<PATH, map<double, pair<Statistics, Statistics>>>> RPE_init();
map<string, map<double, pair<Statistics, Statistics>>> RPE_ALL_init();
map<string, map<PATH, Statistics>> TIME_init();

/// Final print format
void print_latex_ate_rmse_nees_time();    // viz_type == 0
void print_latex_rpe_all_rmse_std();      // viz_type == 1
void print_latex_ate_rmse();              // viz_type == 2
void print_matlab_ate_time_mean_std();    // viz_type == 3
void print_markdown_ate_rmse_nees_time(); // viz_type == 4
void print_matlab_ate_rmse_nees();        // viz_type == 5
void print_latex_rpe_rmse_std();          // viz_type == 6
void print_latex_rpe_median();            // viz_type == 7
void print_latex_ate_1km();               // viz_type == 8

int main(int argc, char **argv) {

  // Basic Setup for path and modes.
  basic_setup(argc, argv);

  // List the ground truth files in this folder
  path_gts = get_path_groundtruths();

  // Get the algorithms we will process
  path_als = get_path_algorithms();

  // create struct of ATE, RPE, Timing, and NEES
  ATE = ATE_init();         // {alg name, {gt, {ori RMSE, pos RMSE}}}
  RPE = RPE_init();         // {alg name, {gt, {ori RMSE, pos RMSE}}}
  RPE_ALL = RPE_ALL_init(); // {alg name, {length, {ori RMSE, pos RMSE}}}
  NEES = NEES_init();       // {alg name, {gt, {ori NEES, pos NEES}}}
  TIME = TIME_init();       // {alg name, {gt, time}}
  GT_DIST = GT_DIST_init(); // {gt, length}

  // Loop through each algorithm type
  for (auto &path_al : path_als) {
#if ROS_AVAILABLE == 2
    if (!rclcpp::ok())
#else
    if (!ros::ok())
#endif
      break;

    // Debug print
    string alg = path_al.filename().string();
    printf("======================================\n");
    printf("[COMP]: %s\n", alg.c_str());

    // Get the list of datasets this algorithm records
    map<string, PATH> datasets = get_list_of_datasets(path_al);

    // Loop through our list of groundtruth datasets, and see if we have it
    for (const auto &path_gt : path_gts) {
      // Check if we have runs for this dataset
      if (datasets.find(path_gt.stem().string()) == datasets.end()) {
        printf(RED "[COMP]: Cannot find runs for %s dataset!!!!!\n" RESET, path_gt.stem().c_str());
        continue;
      }

      // Debug print
      printf("[COMP]: \t %s\n", path_gt.stem().c_str());

      // Loop though and get path to runs for this dataset
      vector<string> path_runs, path_times;
      get_paths_of_run_files(datasets, path_gt, path_runs, path_times);

      /// Compute Timing
      for (const auto &path_time : path_times)
        get_timing(path_time, TIME.at(alg).at(path_gt));
      TIME.at(alg).at(path_gt).calculate();

      for (const auto &path_run : path_runs) {
        // create trajectory to be analyzed
        mins_eval::ResultTrajectory traj(path_run, path_gt.string(), align_mode);

        /// Compute ATE.
        Statistics error_ori, error_pos;
        traj.calculate_ate(error_ori, error_pos);
        ATE.at(alg).at(path_gt).first.values.push_back(error_ori.rmse);
        ATE.at(alg).at(path_gt).second.values.push_back(error_pos.rmse);
        cout << "[COMP]: \t\tATE: " << error_ori.rmse << ", " << error_pos.rmse << ", Complete: " << traj.est_dt() / traj.gt_dt() * 100 << " %" << endl;

        /// Compute RPE.
        map<double, pair<Statistics, Statistics>> error_rpe;
        traj.calculate_rpe(segments, error_rpe);
        for (const auto &err : error_rpe) {
          insert_to_left(RPE_ALL.at(alg).at(err.first).first.values, err.second.first.values);
          insert_to_left(RPE_ALL.at(alg).at(err.first).first.timestamps, err.second.first.timestamps);
          insert_to_left(RPE_ALL.at(alg).at(err.first).second.values, err.second.second.values);
          insert_to_left(RPE_ALL.at(alg).at(err.first).second.timestamps, err.second.second.timestamps);
          insert_to_left(RPE.at(alg).at(path_gt).at(err.first).first.values, err.second.first.values);
          insert_to_left(RPE.at(alg).at(path_gt).at(err.first).first.timestamps, err.second.first.timestamps);
          insert_to_left(RPE.at(alg).at(path_gt).at(err.first).second.values, err.second.second.values);
          insert_to_left(RPE.at(alg).at(path_gt).at(err.first).second.timestamps, err.second.second.timestamps);
        }

        /// Compute NEES
        Statistics nees_ori, nees_pos;
        traj.calculate_nees(nees_ori, nees_pos);
        NEES.at(alg).at(path_gt).first.values.push_back(nees_ori.rmse);
        NEES.at(alg).at(path_gt).second.values.push_back(nees_pos.rmse);

        /// Compute Trajectory Distance
        GT_DIST.at(path_gt) = traj.trajectory_distance();
      }
      ATE.at(alg).at(path_gt).first.calculate();
      ATE.at(alg).at(path_gt).second.calculate();
      NEES.at(alg).at(path_gt).first.calculate();
      NEES.at(alg).at(path_gt).second.calculate();
    }
  }

  //===============================================================================
  //===============================================================================
  //===============================================================================
  // If we have too many datasets to display, segment them for better visualization
  viz_type == 0 ? print_latex_ate_rmse_nees_time() : void();
  viz_type == 1 ? print_latex_rpe_all_rmse_std() : void();
  viz_type == 2 ? print_latex_ate_rmse() : void();
  viz_type == 3 ? print_matlab_ate_time_mean_std() : void();
  viz_type == 4 ? print_markdown_ate_rmse_nees_time() : void();
  viz_type == 5 ? print_matlab_ate_rmse_nees() : void();
  viz_type == 6 ? print_latex_rpe_rmse_std() : void();
  viz_type == 7 ? print_latex_rpe_median() : void();
  viz_type == 8 ? print_latex_ate_1km() : void();

  // Done!
  return EXIT_SUCCESS;
}

/// Read the files and setup for trajectory error evaluation
void basic_setup(int argc, char **argv) {
  // Path to save the plots
  align_mode = argc > 1 ? argv[1] : "";
  folder_groundtruths = argc > 2 ? argv[2] : "";
  folder_algorithms = argc > 3 ? argv[3] : "";

#if ROS_AVAILABLE == 2
  // overwrite options if ROS params exist
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_comparison", options);

  node->get_parameter<std::string>("align_mode", align_mode);
  node->get_parameter<std::string>("path_gts", folder_groundtruths);
  node->get_parameter<std::string>("path_alg", folder_algorithms);
  node->get_parameter<int>("viz_type", viz_type);
#else
  ros::init(argc, argv, "run_comparison");
  ros::param::get("/run_comparison/align_mode", align_mode);
  ros::param::get("/run_comparison/path_gts", folder_groundtruths);
  ros::param::get("/run_comparison/path_alg", folder_algorithms);
  ros::param::get("/run_comparison/viz_type", viz_type);
#endif
}

/// Get paths of the ground truths
VEC_PATH get_path_groundtruths() {
  VEC_PATH path_gts_local;
  for (const auto &p : boost::filesystem::recursive_directory_iterator(folder_groundtruths)) {
    if (p.path().extension() == ".txt") {
      path_gts_local.push_back(p.path());
    }
  }
  sort(path_gts_local.begin(), path_gts_local.end());

  // Try to load our paths
  for (auto &path_groundtruth : path_gts_local) {
    // Load it!
    vector<double> times;
    vector<Eigen::Matrix<double, 7, 1>> poses;
    vector<Eigen::Matrix3d> cov_ori, cov_pos;
    Loader::load_data(path_groundtruth.string(), times, poses, cov_ori, cov_pos);
    // Print its length and stats
    double length = Loader::get_total_length(poses);
    PRINT_DEBUG("[COMP]: %d poses in %s => length of %.2f meters\n", (int)times.size(), path_groundtruth.filename().c_str(), length);
  }
  return path_gts_local;
}

/// Get paths of the estimated trajectory
VEC_PATH get_path_algorithms() {
  // Also create empty statistic objects for each of our datasets
  VEC_PATH path_alg;
  for (const auto &entry : boost::filesystem::directory_iterator(folder_algorithms)) {
    if (boost::filesystem::is_directory(entry)) {
      path_alg.push_back(entry.path());
    }
  }
  sort(path_alg.begin(), path_alg.end());

  // check the files and remove non-estimated trajectory
  // Loop through each algorithm type
  vector<string> found_data;
  for (auto &path : path_alg) {
    // Get the list of datasets this algorithm records
    map<string, PATH> path_algo_datasets;
    for (auto &entry : boost::filesystem::directory_iterator(path)) {
      if (boost::filesystem::is_directory(entry)) {
        found_data.push_back(entry.path().filename().string());
        path_algo_datasets.insert({entry.path().filename().string(), entry.path()});
      }
    }
  }

  // Erase ground trajectory if none of the estimated trajectory exist
  for (auto it = path_gts.begin(); it != path_gts.end();) {
    if (find(found_data.begin(), found_data.end(), (*it).stem().string()) == found_data.end())
      it = path_gts.erase(it);
    else
      ++it;
  }
  return path_alg;
}

/// Initialize the ATE record
map<string, map<PATH, pair<Statistics, Statistics>>> ATE_init() {
  map<string, map<PATH, pair<Statistics, Statistics>>> algo_ate;
  for (const auto &p : path_als) {
    map<PATH, pair<Statistics, Statistics>> temp;
    for (const auto &path_gt : path_gts) {
      temp.insert({path_gt, {Statistics(), Statistics()}});
    }
    algo_ate.insert({p.filename().string(), temp});
  }
  return algo_ate;
}

/// Initialize the Ground truth distance record
map<PATH, double> GT_DIST_init() {
  map<PATH, double> gt_dist;
  for (const auto &path_gt : path_gts) {
    gt_dist.insert({path_gt, double()});
  }
  return gt_dist;
}

/// Initialize the RPE record
map<string, map<PATH, map<double, pair<Statistics, Statistics>>>> RPE_init() {
  map<string, map<PATH, map<double, pair<Statistics, Statistics>>>> rpe;
  for (const auto &p : path_als) {
    map<PATH, map<double, pair<Statistics, Statistics>>> temp;
    for (const auto &path_gt : path_gts) {
      map<double, pair<Statistics, Statistics>> temp2;
      for (const auto &len : segments) {
        temp2.insert({len, {Statistics(), Statistics()}});
      }
      temp.insert({path_gt, temp2});
    }
    rpe.insert({p.filename().string(), temp});
  }
  return rpe;
}

/// Initialize the RPE ALL record
map<string, map<double, pair<Statistics, Statistics>>> RPE_ALL_init() {
  map<string, map<double, pair<Statistics, Statistics>>> rpe;
  for (const auto &p : path_als) {
    map<double, pair<Statistics, Statistics>> temp;
    for (const auto &len : segments) {
      temp.insert({len, {Statistics(), Statistics()}});
    }
    rpe.insert({p.filename().string(), temp});
  }
  return rpe;
}

/// Initialize the timing record
map<string, map<PATH, Statistics>> TIME_init() {
  map<string, map<PATH, Statistics>> time;
  for (const auto &p : path_als) {
    map<PATH, Statistics> temp;
    for (const auto &path_gt : path_gts) {
      temp.insert({path_gt, Statistics()});
    }
    time.insert({p.filename().string(), temp});
  }
  return time;
}

/// Initialize the NEES record
map<string, map<PATH, pair<Statistics, Statistics>>> NEES_init() {
  map<string, map<PATH, pair<Statistics, Statistics>>> nees;
  for (const auto &p : path_als) {
    map<PATH, pair<Statistics, Statistics>> temp;
    for (const auto &path_gt : path_gts) {
      temp.insert({path_gt, {Statistics(), Statistics()}});
    }
    nees.insert({p.filename().string(), temp});
  }
  return nees;
}

/// Get files given directory path
map<string, PATH> get_list_of_datasets(const PATH &path) {
  map<string, PATH> path_algo_datasets;
  for (auto &entry : boost::filesystem::directory_iterator(path)) {
    if (boost::filesystem::is_directory(entry)) {
      path_algo_datasets.insert({entry.path().filename().string(), entry.path()});
    }
  }
  return path_algo_datasets;
}

/// search for specific extentioned files for analysis.
void get_paths_of_run_files(map<string, PATH> datasets, const PATH &path_gt, vector<string> &run_paths, vector<string> &time_paths) {
  for (auto &entry : boost::filesystem::directory_iterator(datasets.at(path_gt.stem().string()))) {
    // Files containing timing record
    if (entry.path().extension() == ".time") {
      time_paths.push_back(entry.path().string());
      continue;
    }
    // Files containing estimation record
    if (entry.path().extension() == ".txt") {
      run_paths.push_back(entry.path().string());
      continue;
    }
    // This is old extension version of MINS time record.
    if (entry.path().extension() == ".record") {
      time_paths.push_back(entry.path().string());
      continue;
    }
  }
  sort(run_paths.begin(), run_paths.end());
}

/// Read timing record
void get_timing(const string time_path, Statistics &time_) {
  // Try to open our trajectory file
  ifstream file(time_path);
  if (!file.is_open()) {
    PRINT_ERROR(RED "[LOAD]: Unable to open timing file: %s\n" RESET, time_path.c_str());
    exit(EXIT_FAILURE);
  }

  // read
  string reading;
  while (getline(file, reading)) {
    time_.values.push_back(stod(reading));
  }
}

/// Find the best ATE for each dataset
map<PATH, pair<double, double>> get_best_ATE(VEC_PATH v_gt, map<string, map<PATH, pair<Statistics, Statistics>>> ATE) {
  map<PATH, pair<double, double>> best;
  for (const auto &gt : v_gt) {
    best.insert({gt, {INFINITY, INFINITY}});
    for (const auto &alg : ATE) {
      auto stat = alg.second.at(gt);
      if (stat.first.values.empty() || stat.second.values.empty())
        continue;
      best.at(gt).first = stat.first.mean < best.at(gt).first ? stat.first.mean : best.at(gt).first;
      best.at(gt).second = stat.second.mean < best.at(gt).second ? stat.second.mean : best.at(gt).second;
    }
  }
  return best;
}

/// Print ATE RMSE, NEES, and TIME in Latex table format
void print_latex_ate_rmse_nees_time() {
  printf("\n\n\n");
  printf("============================================\n");
  printf("LATEX ATE TABLE \n");
  printf("============================================\n");
  // If we have too many datasets to display, segment them for better visualization
  vector<VEC_PATH> vv_gt;
  for (int i = 0; i < 100; i++)
    vv_gt.emplace_back();

  // Segment the ground truth
  for (int i = 0; i < (int)path_gts.size(); i++)
    vv_gt.at(i / max_col).push_back(path_gts.at(i));

  for (const auto &v_gt : vv_gt) {
    if (v_gt.empty())
      break;
    // Table header
    string ls = "c";
    for (auto gt : v_gt)
      ls += "|ccc";
    printf("\\begin{table}[H]\n \\begin{adjustbox}{width=%.1f\\textwidth,center}\n  \\begin{tabular}{%s}\n   \\toprule\n   ", (double)v_gt.size() / max_col, ls.c_str());

    // Name of the dataset with multi-column
    for (const auto &g : v_gt)
      printf("&\\multicolumn{3}{c%s}{\\textbf{%s}}", g == v_gt.back() ? "" : "|", replace_all_copy(g.stem().string(), "_", "\\_").c_str());
    printf("\\\\\n   ");

    // RMSE, NEES, and/or Time header
    for (int i = 0; i < (int)v_gt.size(); i++)
      printf(" & \\textbf{RMSE (deg / m)} & \\textbf{NEES} & \\textbf{Time (s)}");
    printf("\\\\ \\midrule\n");

    // Find the best RMSE results to emphasize!
    map<PATH, pair<double, double>> best = get_best_ATE(v_gt, ATE);

    // Contents
    for (const auto &alg : ATE) {
      printf("%s", replace_all_copy(alg.first, "_", "\\_").c_str());
      for (const auto &gt : v_gt) {
        // RMSE
        auto rmse = alg.second.at(gt);
        if (rmse.first.values.empty() || rmse.second.values.empty())
          printf(" & - / -");
        else {
          rmse.first.mean == best.at(gt).first ? printf(" & \\textbf{%.3f}", rmse.first.mean) : printf(" & %.3f", rmse.first.mean);
          printf(" $\\pm$ %.3f", rmse.first.std);
          rmse.second.mean == best.at(gt).second ? printf(" / \\textbf{%.3f}", rmse.second.mean) : printf(" / %.3f", rmse.second.mean);
          printf(" $\\pm$ %.3f", rmse.second.std);
        }
        // NEES
        auto nees = NEES.at(alg.first).at(gt);
        nees.first.mean < 4 ? printf(" & %.1f", nees.first.mean) : printf(" & \\colorbox{orange}{%.1f}", nees.first.mean);
        printf(" $\\pm$ %.1f", nees.first.std);
        nees.second.mean < 4 ? printf(" / %.1f", nees.second.mean) : printf(" / \\colorbox{orange}{%.1f}", nees.second.mean);
        printf(" $\\pm$ %.1f", nees.second.std);
        // Time
        printf(" & %.1f $\\pm$ %.1f", TIME.at(alg.first).at(gt).mean, TIME.at(alg.first).at(gt).std);
      }
      alg.first != ATE.rbegin()->first ? printf(" \\\\ \\midrule \n") : printf(" \\\\ \\bottomrule \n");
    }
    printf("  \\end{tabular}\n \\end{adjustbox}\n\\end{table}\n\\vspace{-0.8cm}\n\n");
  }
}

/// Print ATE RMSE in Latex table format
void print_latex_ate_rmse() {
  printf("\n\n\n");
  printf("============================================\n");
  printf("LATEX ATE TABLE (RMSE)\n");
  printf("============================================\n");
  vector<VEC_PATH> vv_gt;
  for (int i = 0; i < 100; i++)
    vv_gt.emplace_back();

  // Segment the ground truth
  for (int i = 0; i < (int)path_gts.size(); i++)
    vv_gt.at(i / max_col).push_back(path_gts.at(i));

  for (const auto &v_gt : vv_gt) {
    if (v_gt.empty())
      break;
    // Table header
    string ls = "c";
    for (auto gt : v_gt)
      ls += "c";
    printf("\\begin{table}[H]\n \\begin{adjustbox}{width=%.1f\\textwidth,center}\n  \\begin{tabular}{%s}\n   \\toprule\n   ", (double)v_gt.size() / max_col, ls.c_str());

    // Name of the dataset with multi-column
    for (const auto &g : v_gt)
      printf("&\\textbf{%s}", replace_all_copy(g.stem().string(), "_", "\\_").c_str());
    printf("\\\\\n   ");

    // RMSE, NEES, and/or Time header
    for (int i = 0; i < (int)v_gt.size(); i++)
      printf(" & \\textbf{RMSE (deg / m)}");
    printf("\\\\ \\midrule\n");

    // Contents
    for (const auto &alg : ATE) {
      printf("%s", replace_all_copy(alg.first, "_", "\\_").c_str());
      for (const auto &gt : v_gt) {
        // RMSE
        auto rmse = alg.second.at(gt);
        if (rmse.first.values.empty() || rmse.second.values.empty()) {
          printf(" & - / -");
        } else {
          printf(" & %.3f", rmse.first.mean);
          printf(" / %.3f", rmse.second.mean);
        }
      }
      alg.first != ATE.rbegin()->first ? printf(" \\\\ \\midrule \n") : printf(" \\\\ \\bottomrule \n");
    }
    printf("  \\end{tabular}\n \\end{adjustbox}\n\\end{table}\n\\vspace{-0.8cm}\n\n");
  }
}

/// Print ATE RMSE divided by the length of the trajectory (unit 1km) in Latex table format
void print_latex_ate_1km() {
  printf("\n\n\n");
  printf("============================================\n");
  printf("LATEX ATE TABLE\n");
  printf("============================================\n");
  // Table header
  string ls = "c";
  for (auto gt : path_gts)
    ls += "c";
  printf("\\begin{table*}[t]\n \\begin{adjustbox}{width=1\\textwidth,center}\n  \\begin{tabular}{%s}\n   \\toprule\n   ", ls.c_str());

  // Name of the dataset with multi-column
  for (const auto &gt : path_gts)
    printf("&\\textbf{%s}", replace_all_copy(gt.stem().string(), "_", "\\_").c_str());
  printf("\\\\\n   ");

  for (const auto &alg : ATE) {
    printf("%s", replace_all_copy(alg.first, "_", "\\_").c_str());
    for (const auto &path_gt : path_gts) {
      // RMSE
      auto rmse = alg.second.at(path_gt);
      if (rmse.second.values.empty()) {
        printf(" & -");
      } else {
        assert(GT_DIST.at(path_gt) != 0);
        printf(" & %.2f", rmse.second.mean / GT_DIST.at(path_gt) * 1000); // per 1km
                                                                          //        printf(" & %.2f", rmse.second.mean );
      }
    }
    printf(" \\\\\n");
  }
  printf("  \\end{tabular}\n \\end{adjustbox}\n\\end{table*}\n\n\n");
}

/// Print RPE RMSE in Latex table format
void print_latex_rpe_rmse_std() {
  assert(segments.size() == 1);
  printf("\n\n\n");
  // Finally print the RPE for all the runs
  printf("============================================\n");
  printf("RPE LATEX TABLE (%.2f m)\n", segments.at(0));
  printf("============================================\n");
  vector<VEC_PATH> vv_gt;
  for (int i = 0; i < 100; i++)
    vv_gt.emplace_back();

  // Segment the ground truth
  for (int i = 0; i < (int)path_gts.size(); i++)
    vv_gt.at(i / max_col).push_back(path_gts.at(i));

  for (const auto &v_gt : vv_gt) {
    if (v_gt.empty())
      break;
    // Table header
    string ls = "c";
    for (auto gt : v_gt)
      ls += "c";
    printf("\\begin{table*}[t]\n \\begin{adjustbox}{width=%.1f\\textwidth,center}\n  \\begin{tabular}{%s}\n   \\toprule\n   ", (double)v_gt.size() / max_col, ls.c_str());

    // Name of the dataset with multi-column
    for (const auto &g : v_gt)
      printf("&\\textbf{%s}", replace_all_copy(g.stem().string(), "_", "\\_").c_str());
    printf("\\\\\n   ");

    // RMSE, NEES, and/or Time header
    //    for (int i = 0; i < (int)v_gt.size(); i++)
    //      printf(" & \\textbf{RMSE (deg / m)}");
    //    printf("\\\\ \\midrule\n");

    // Contents
    for (const auto &alg : RPE) {
      printf("%s", replace_all_copy(alg.first, "_", "\\_").c_str());
      for (const auto &gt : v_gt) {
        // RMSE
        auto rmse = alg.second.at(gt).at(segments.front());
        rmse.first.calculate();
        rmse.second.calculate();
        if (rmse.first.values.empty() || rmse.second.values.empty()) {
          printf(" & - / -");
        } else {
          printf(" & %.3f", rmse.first.mean);
          printf(" $\\pm$ %.3f", rmse.first.std);
          printf(" / %.3f", rmse.second.mean);
          printf(" $\\pm$ %.3f", rmse.second.std);
        }
      }
      alg.first != ATE.rbegin()->first ? printf(" \\\\ \\midrule \n") : printf(" \\\\ \\bottomrule \n");
    }
    printf("  \\end{tabular}\n \\end{adjustbox}\n\\end{table*}\n\n");
  }
}

/// Print RPE median error in Latex table format
void print_latex_rpe_median() {
  assert(segments.size() == 1);
  printf("\n\n\n");
  // Finally print the RPE for all the runs
  printf("============================================\n");
  printf("RPE MEDIAN LATEX TABLE (%.2f m)\n", segments.at(0));
  printf("============================================\n");
  vector<VEC_PATH> vv_gt;
  for (int i = 0; i < 100; i++)
    vv_gt.emplace_back();

  // Segment the ground truth
  for (int i = 0; i < (int)path_gts.size(); i++)
    vv_gt.at(i / max_col).push_back(path_gts.at(i));

  for (const auto &v_gt : vv_gt) {
    if (v_gt.empty())
      break;
    // Table header
    string ls = "c";
    for (auto gt : v_gt)
      ls += "c";
    printf("\\begin{table*}[t]\n \\begin{adjustbox}{width=%.1f\\textwidth,center}\n  \\begin{tabular}{%s}\n   \\toprule\n   ", (double)v_gt.size() / max_col, ls.c_str());

    // Name of the dataset with multi-column
    for (const auto &g : v_gt)
      printf("&\\textbf{%s}", replace_all_copy(g.stem().string(), "_", "\\_").c_str());
    printf("\\\\\n   ");

    // RMSE, NEES, and/or Time header
    //    for (int i = 0; i < (int)v_gt.size(); i++)
    //      printf(" & \\textbf{RMSE (deg / m)}");
    //    printf("\\\\ \\midrule\n");

    // Contents
    for (const auto &alg : RPE) {
      printf("%s", replace_all_copy(alg.first, "_", "\\_").c_str());
      for (const auto &gt : v_gt) {
        // RMSE
        auto rmse = alg.second.at(gt).at(segments.front());
        rmse.first.calculate();
        rmse.second.calculate();
        if (rmse.first.values.empty() || rmse.second.values.empty()) {
          printf(" & - / -");
        } else {
          printf(" & %.3f", rmse.first.median);
          printf(" / %.3f", rmse.second.median);
        }
      }
      alg.first != ATE.rbegin()->first ? printf(" \\\\ \\midrule \n") : printf(" \\\\ \\bottomrule \n");
    }
    printf("  \\end{tabular}\n \\end{adjustbox}\n\\end{table*}\n\n");
  }
}

/// Print RPE RMSE of all trajecotry (no distiction between trajectories) in Latex table format
void print_latex_rpe_all_rmse_std() {
  printf("\n\n\n");
  // Finally print the RPE for all the runs
  printf("============================================\n");
  printf("RPE LATEX TABLE\n");
  printf("============================================\n");
  for (const auto &len : segments)
    printf(" & \\textbf{%dm}", (int)len);
  printf(" \\\\\n");
  for (auto &al : RPE_ALL) {
    printf("%s", replace_all_copy(al.first, "_", "\\_").c_str());
    for (auto &seg : al.second) {
      seg.second.first.calculate();
      seg.second.second.calculate();

      printf(" & %.3f", seg.second.first.mean);
      printf(" $\\pm$ %.3f", seg.second.first.std);
      printf(" / %.3f", seg.second.second.mean);
      printf(" $\\pm$ %.3f", seg.second.second.std);
    }
    printf(" \\\\\n");
  }
}

/// Print ATE and time in matlab format
void print_matlab_ate_time_mean_std() {

  printf("\n\n\n");
  printf("============================================\n");
  printf("MATLAB VARIABLES\n");
  printf("============================================\n");
  struct stat {
    struct mean_std {
      double mean;
      double std;
    };
    mean_std rmse_ori;
    mean_std rmse_pos;
    mean_std nees_ori;
    mean_std nees_pos;
    mean_std time;
  };
  map<string, map<string, stat>> name_alg_stat;
  bool MATLAB_find = false;
  for (const auto &path_gt : path_gts) {
    for (auto &alg : ATE) {
      // Parse the name
      string name_full = alg.first.c_str();
      int id = 0;
      for (; id < (int)name_full.length(); id++) {
        if (isdigit(name_full[id])) {
          MATLAB_find = true;
          break;
        }
      }
      if (!MATLAB_find)
        break;
      string name = name_full.substr(0, id);
      string number = name_full.substr(id);
      string name_traj = name + path_gt.stem().string().c_str();

      name_alg_stat[name_traj][number].rmse_ori.mean = alg.second.at(path_gt).first.mean;
      name_alg_stat[name_traj][number].rmse_ori.std = alg.second.at(path_gt).first.std;
      name_alg_stat[name_traj][number].rmse_pos.mean = alg.second.at(path_gt).second.mean;
      name_alg_stat[name_traj][number].rmse_pos.std = alg.second.at(path_gt).second.std;
      name_alg_stat[name_traj][number].nees_ori.mean = NEES.at(alg.first).at(path_gt).first.mean;
      name_alg_stat[name_traj][number].nees_ori.std = NEES.at(alg.first).at(path_gt).first.std;
      name_alg_stat[name_traj][number].nees_pos.mean = NEES.at(alg.first).at(path_gt).second.mean;
      name_alg_stat[name_traj][number].nees_pos.std = NEES.at(alg.first).at(path_gt).second.std;
      name_alg_stat[name_traj][number].time.mean = TIME.at(alg.first).at(path_gt).mean;
      name_alg_stat[name_traj][number].time.std = TIME.at(alg.first).at(path_gt).std;
    }
  }
  assert(MATLAB_find);
  for (auto name : name_alg_stat) {
    printf("%s_rmse_ori_mean = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.rmse_ori.mean);
    }
    printf("\b\b};\n");
    printf("%s_rmse_ori_std  = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.rmse_ori.std);
    }
    printf("\b\b};\n");
    printf("%s_rmse_pos_mean = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.rmse_pos.mean);
    }
    printf("\b\b};\n");
    printf("%s_rmse_pos_std  = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.rmse_pos.std);
    }
    printf("\b\b};\n");
    printf("%s_nees_ori_mean = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.nees_ori.mean);
    }
    printf("\b\b};\n");
    printf("%s_nees_ori_std  = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.nees_ori.std);
    }
    printf("\b\b};\n");
    printf("%s_nees_pos_mean = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.nees_pos.mean);
    }
    printf("\b\b};\n");
    printf("%s_nees_pos_std  = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.nees_pos.std);
    }
    printf("\b\b};\n");
    printf("%s_time_mean = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.time.mean);
    }
    printf("\b\b};\n");
    printf("%s_time_std  = {", name.first.c_str());
    for (const auto &alg : name.second) {
      printf("'%s', ", replace_all_copy(alg.first, "_", "\\_").c_str());
    }
    printf("\b\b; ");
    for (const auto &alg : name.second) {
      printf("%.4f, ", alg.second.time.std);
    }
    printf("\b\b};\n");
  }
}

/// Print ATE in matlab format
void print_matlab_ate_rmse_nees() {
  for (const auto &alg : ATE) {
    // Name of alg
    printf("%s", alg.first.c_str());

    // supports only 1 ground truth...
    assert(path_gts.size() == 1);

    // should have all valid values...
    assert(!alg.second.at(path_gts[0]).first.values.empty() && !alg.second.at(path_gts[0]).second.values.empty());
    // RMSE
    printf(" %.3f %.3f", alg.second.at(path_gts[0]).first.mean, alg.second.at(path_gts[0]).second.mean);
    // NEES
    printf(" %.3f %.3f\n", NEES.at(alg.first).at(path_gts[0]).first.mean, NEES.at(alg.first).at(path_gts[0]).second.mean);
  }
}

/// Print ATE, NEES, and time in Markdown format (giothub)
void print_markdown_ate_rmse_nees_time() {
  vector<VEC_PATH> vv_gt;
  for (int i = 0; i < 100; i++)
    vv_gt.emplace_back();

  // Segment the ground truth
  for (int i = 0; i < (int)path_gts.size(); i++)
    vv_gt.at(i / max_col).push_back(path_gts.at(i));

  printf("\n\n\n");
  printf("============================================\n");
  printf("ATE MARKDOWN RENDERING\n");
  printf("============================================\n");
  setlocale(LC_ALL, "en_US.UTF-8");
  for (const auto &v_gt : vv_gt) {
    if (v_gt.empty())
      break;
    printf("<table>\n <tr><th><sub></sub></th>\n"); // an empty column for algorithm names
    for (auto &gt : v_gt)
      printf("<sub><th colspan=\"3\"><sub><b>%s</b></sub></th></sub>\n", gt.stem().string().c_str());
    printf("</tr>\n");
    // Headers
    printf("<tr><th></th>\n"); // an empty column for algorithm names
    for (int i = 0; i < (int)v_gt.size(); i++) {
      printf("<td align=\"center\"><sub><b> RMSE (deg / m) </b></sub></td>\n");
      printf("<td align=\"center\"><sub><b>      NEES      </b></sub></td>\n");
      printf("<td align=\"center\"><sub><b>    Time (s)    </b></sub></td>\n");
    }
    printf("</tr>\n");

    // Find the best RMSE results to emphasize!
    map<PATH, pair<double, double>> best = get_best_ATE(v_gt, ATE);

    // Each algorithm RMSE, NEES, Time
    for (auto &alg : ATE) {
      printf("<tr><td align=\"center\"><sub><b>%s</b></sub></td>", alg.first.c_str());
      for (const auto &gt : v_gt) {
        // RMSE
        auto rmse = alg.second.at(gt);
        if (rmse.first.values.empty() || rmse.second.values.empty()) {
          printf("<td align=\"center\"><sub>- / -</sub></td>");
        } else {
          printf("<td align=\"center\"><sub>");
          rmse.first.mean == best.at(gt).first ? printf("<b>%.3f</b>", rmse.first.mean) : printf("%.3f", rmse.first.mean);
          printf(" %lc %.3f / ", 0x0B1, rmse.first.std);
          rmse.second.mean == best.at(gt).second ? printf("<b>%.3f</b>", rmse.second.mean) : printf("%.3f", rmse.second.mean);
          printf(" %lc %.3f ", 0x0B1, rmse.second.std);
          printf("</sub></td>");
        }
        // NEES
        auto nees = NEES.at(alg.first).at(gt);
        printf("<td align=\"center\"><sub>%.1f %lc %.1f ", nees.first.mean, 0x0B1, nees.first.std);
        printf("/ %.1f %lc %.1f</sub></td>", nees.second.mean, 0x0B1, nees.second.std);
        // Time
        auto time = TIME.at(alg.first).at(gt);
        printf("<td align=\"center\"><sub>%.1f %lc %.1f</sub></td>", time.mean, 0x0B1, time.std);
      }
      printf("</tr>\n");
    }
    printf("</table>\n");
  }
}
