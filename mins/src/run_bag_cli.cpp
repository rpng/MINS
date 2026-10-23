/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Plays a ROS1 bag through the estimator without ROS: same topics, conversions and play order as run_bag.cpp,
// reading the bag with RosbagReader and writing results through State_Logger.

#include "core/SystemManager.h"
#include "io/RosMsgs.h"
#include "io/RosbagReader.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSystem.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/PointCloud2Convert.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include <cmath>
#include <deque>

using namespace std;
using namespace mins;

namespace {

shared_ptr<Options> op;
shared_ptr<SystemManager> sys;

struct Item {
  string topic, type;
  uint64_t time; // record time, ns
  vector<uint8_t> data;
  double stamp; // header stamp, filled for camera messages
  bool used;    // consumed early as the second image of a stereo pair
};

// run_bag pairs a stereo image with a partner that may be recorded later, so keep this much of the bag ahead of play
constexpr uint64_t LOOKAHEAD_NS = 2000000000ull;

deque<Item> ahead;
vector<deque<double>> cam_seen; // header stamps of already played images per camera, for the pairing search

int cam_index(const string &topic) {
  for (int i = 0; i < op->est->cam->max_n; i++)
    if (op->est->cam->topic.at(i) == topic)
      return i;
  return -1;
}

// Same as ROSHelper::Image2Data, appending to cam
void add_image(const Item &it, int cam_id, ov_core::CameraData &cam) {
  double stamp;
  cv::Mat img = rosmsg::image(it.type, it.data, stamp);
  cam.timestamp = it.stamp;
  cam.sensor_ids.push_back(cam_id);
  cam.images.push_back(img);
  cam.masks.push_back(op->est->cam->use_mask.at(cam_id) ? op->est->cam->masks.at(cam_id) : cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
}

// run_bag's find_stereo_pair over every image of the partner camera: closest stamp, accepted within 10 ms and only if
// it has not been played yet
Item *find_stereo_pair(double meas_t, int stereo_id) {
  map<double, Item *> stamps;
  for (double t : cam_seen.at(stereo_id))
    stamps.insert({t, nullptr});
  for (auto &it : ahead)
    if (it.topic == op->est->cam->topic.at(stereo_id))
      stamps.insert({it.stamp, &it});
  if (stamps.empty())
    return nullptr;
  auto lb = stamps.lower_bound(meas_t);
  if (lb == stamps.end())
    lb--;
  else if (lb != stamps.begin()) {
    auto prev = std::prev(lb);
    lb = abs(meas_t - prev->first) < abs(meas_t - lb->first) ? prev : lb;
  }
  if (lb->second == nullptr || abs(lb->first - meas_t) >= 0.01)
    return nullptr;
  return lb->second;
}

void play_camera(Item &it, int cam_id) {
  cam_seen.at(cam_id).push_back(it.stamp);
  while (cam_seen.at(cam_id).size() > 200)
    cam_seen.at(cam_id).pop_front();

  ov_core::CameraData cam;
  auto pair = op->est->cam->stereo_pairs.find(cam_id);
  if (pair == op->est->cam->stereo_pairs.end()) {
    add_image(it, cam_id, cam);
    PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d\n" RESET, it.stamp, cam_id);
    sys->feed_measurement_camera(cam);
    return;
  }
  Item *other = find_stereo_pair(it.stamp, pair->second);
  if (other == nullptr) {
    PRINT4(RED "Cannot find proper stereo pair of CAM%d!\n" RESET, cam_id);
    return;
  }
  other->used = true;
  cam_seen.at(pair->second).push_back(other->stamp);
  add_image(it, cam_id, cam);
  add_image(*other, pair->second, cam);
  PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, it.stamp, cam_id, pair->second);
  sys->feed_measurement_camera(cam);
}

void play(Item &it, const shared_ptr<State_Logger> &save) {
  if (it.used)
    return;
  auto est = op->est;

  if (it.topic == est->imu->topic) {
    ov_core::ImuData imu = rosmsg::imu(it.data);
    PRINT1(GREEN "[BAG] IMU measurement: %.3f" RESET, imu.timestamp);
    PRINT1(GREEN "|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
    if (sys->feed_measurement_imu(imu) && op->sys->save_trajectory)
      save->save_trajectory_to_file(sys);
    return;
  }

  if (est->cam->enabled) {
    int cam_id = cam_index(it.topic);
    if (cam_id >= 0) {
      play_camera(it, cam_id);
      return;
    }
  }

  if (est->wheel->enabled && it.topic == est->wheel->topic) {
    WheelData data;
    if (it.type == "sensor_msgs/JointState") {
      JointState js = rosmsg::jointstate(it.data);
      if (js.name.empty() || js.name.at(0) != est->wheel->sub_topics.at(0)) {
        PRINT4(RED "Wheel type setting is wrong!\n" RESET);
        exit(EXIT_FAILURE);
      }
      data.time = js.stamp;
      data.m1 = js.velocity.at(0);
      data.m2 = js.velocity.at(1);
    } else {
      Odometry od = rosmsg::odometry(it.data);
      data.time = od.stamp;
      data.m1 = od.angular(2);
      data.m2 = od.linear(0);
    }
    PRINT1(MAGENTA "[BAG] WHL measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
    sys->feed_measurement_wheel(data);
    return;
  }

  if (est->gps->enabled) {
    for (int id = 0; id < est->gps->max_n; id++) {
      if (it.topic != est->gps->topic.at(id))
        continue;
      bool is_fix = it.type == "sensor_msgs/NavSatFix";
      GPSData data;
      data.id = id;
      if (is_fix) {
        NavSatFix fix = rosmsg::navsatfix(it.data);
        data.time = fix.stamp;
        data.meas << fix.latitude, fix.longitude, fix.altitude;
        data.noise << fix.covariance[0], fix.covariance[4], fix.covariance[8];
      } else {
        StampedPose p = rosmsg::pose(it.type, it.data);
        data.time = p.stamp;
        data.meas = p.p;
        data.noise = est->gps->noise * Eigen::Vector3d::Ones();
      }
      // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
      data.noise(0) <= 0.0 || est->gps->overwrite_noise ? data.noise(0) = est->gps->noise : double();
      data.noise(1) <= 0.0 || est->gps->overwrite_noise ? data.noise(1) = est->gps->noise : double();
      data.noise(2) <= 0.0 || est->gps->overwrite_noise ? data.noise(2) = est->gps->noise * 2 : double();
      PRINT1(CYAN "[BAG] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
      PRINT1(CYAN "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
      sys->feed_measurement_gps(data, is_fix);
      return;
    }
  }

  if (est->lidar->enabled) {
    for (int id = 0; id < est->lidar->max_n; id++) {
      if (it.topic != est->lidar->topic.at(id))
        continue;
      PointCloud2 msg = rosmsg::pointcloud2(it.data);
      auto data = fromPC2(msg);
      data->header.frame_id = to_string(id);
      data->header.stamp = msg.stamp * 1000; // in msec, truncated like ROSHelper::rosPC2pclPC
      PRINT1("[BAG] LDR measurement: %.3f|%d|%d\n", (double)data->header.stamp / 1000, id, data->points.size());
      sys->feed_measurement_lidar(data);
      return;
    }
  }

  if (est->vicon->enabled) {
    for (int id = 0; id < est->vicon->max_n; id++) {
      if (it.topic != est->vicon->topic.at(id))
        continue;
      StampedPose p = rosmsg::pose(it.type, it.data);
      ViconData data;
      data.time = p.stamp;
      data.id = id;
      data.pose.block(0, 0, 3, 1) = ov_core::log_so3(ov_core::quat_2_Rot(p.q));
      data.pose.block(3, 0, 3, 1) = p.p;
      PRINT1("[BAG] VCN measurement: %.3f|%d|", data.time, data.id);
      PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", data.pose(0), data.pose(1), data.pose(2), data.pose(3), data.pose(4), data.pose(5));
      sys->feed_measurement_vicon(data);
      return;
    }
  }
}

// ros::Time + ros::Duration(sec), in ns
uint64_t add_seconds(uint64_t t, double sec) {
  double whole = floor(sec);
  return t + static_cast<int64_t>(whole) * 1000000000ll + static_cast<int64_t>(round((sec - whole) * 1e9));
}

} // namespace

int run(int argc, char **argv) {
  if (argc < 2) {
    PRINT4(RED "usage: %s <config.yaml> [bag, overrides the config's path_bag; /dev/stdin plays an unindexed bag from a pipe]\n" RESET, argv[0]);
    return EXIT_FAILURE;
  }
  auto parser = make_shared<ov_core::YamlParser>(argv[1]);
  op = make_shared<Options>();
  op->load_print(parser);
  op->sys->save_prints ? Print_Logger::open_file(op->sys->path_state, true) : void();
  sys = make_shared<SystemManager>(op->est);
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    return EXIT_FAILURE;
  }

  // One config for many bags, as the sys_path_bag ROS param does for run_bag
  if (argc > 2)
    op->sys->path_bag = argv[2];

  RosbagReader bag(op->sys->path_bag);
  set<string> topics = {op->est->imu->topic};
  if (op->est->cam->enabled)
    topics.insert(op->est->cam->topic.begin(), op->est->cam->topic.begin() + op->est->cam->max_n);
  if (op->est->wheel->enabled)
    topics.insert(op->est->wheel->topic);
  if (op->est->gps->enabled)
    topics.insert(op->est->gps->topic.begin(), op->est->gps->topic.begin() + op->est->gps->max_n);
  if (op->est->lidar->enabled)
    topics.insert(op->est->lidar->topic.begin(), op->est->lidar->topic.begin() + op->est->lidar->max_n);
  if (op->est->vicon->enabled)
    topics.insert(op->est->vicon->topic.begin(), op->est->vicon->topic.begin() + op->est->vicon->max_n);

  // Same play window as run_bag, including how it clamps bag_durr
  uint64_t time_init = add_seconds(bag.start_time(), op->sys->bag_start);
  double end_sec = static_cast<double>(bag.end_time() / 1000000000ull) + 1e-9 * static_cast<double>(bag.end_time() % 1000000000ull);
  op->sys->bag_durr = end_sec < op->sys->bag_durr ? end_sec : op->sys->bag_durr;
  uint64_t time_finish = op->sys->bag_durr < 0 ? bag.end_time() : add_seconds(time_init, op->sys->bag_durr);

  auto save = make_shared<State_Logger>(op);
  cam_seen.resize(op->est->cam->max_n);
  bool finished = false;
  size_t count = 0;
  auto drain = [&](uint64_t until) {
    while (!ahead.empty() && ahead.front().time + LOOKAHEAD_NS <= until) {
      Item it = std::move(ahead.front());
      ahead.pop_front();
      if (it.time > time_finish) {
        finished = true;
        return;
      }
      if (it.time >= time_init)
        play(it, save);
    }
  };

  bag.read(topics, [&](const RosbagReader::Message &m) {
    bool is_cam = op->est->cam->enabled && cam_index(m.conn->topic) >= 0;
    // In case the image does not have timestamp (then 0), overwrite it with message time, as run_bag does
    double stamp = is_cam ? rosmsg::stamp(m.data) : 0;
    if (is_cam && stamp == 0)
      stamp = static_cast<double>(m.time / 1000000000ull) + 1e-9 * static_cast<double>(m.time % 1000000000ull);
    count++;
    ahead.push_back({m.conn->topic, m.conn->type, m.time, m.data, stamp, false});
    drain(m.time);
    return !finished;
  });
  drain(UINT64_MAX);
  if (count == 0) {
    PRINT4(RED "No messages to play on specified topics. Exiting.\n" RESET);
    return EXIT_FAILURE;
  }

  sys->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();
  return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
  // A bad bag or message throws; report it instead of aborting with the logs unflushed
  try {
    return run(argc, argv);
  } catch (const std::exception &e) {
    PRINT4(RED "%s\n" RESET, e.what());
    return EXIT_FAILURE;
  }
}
