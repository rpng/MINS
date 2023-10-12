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

#include "ROSSubscriber.h"
#include "ROSHelper.h"
#include "ROSPublisher.h"
#include "SystemManager.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "update/gps/GPSTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"

using namespace std;
using namespace Eigen;
using namespace mins;

ROSSubscriber::ROSSubscriber(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<SystemManager> sys, std::shared_ptr<ROSPublisher> pub) : nh(nh), sys(sys), pub(pub) {
  // Copy option for easier access
  op = sys->state->op;

  // Create imu subscriber (handle legacy ros param info)
  subs.push_back(nh->subscribe(op->imu->topic, 1000, &ROSSubscriber::callback_inertial, this));
  PRINT2("subscribing to imu: %s\n", op->imu->topic.c_str());

  // Create camera subscriber
  if (op->cam->enabled) {
    for (int i = 0; i < op->cam->max_n; i++) {
      // Check if this is stereo
      if (op->cam->stereo_pairs.find(i) != op->cam->stereo_pairs.end()) {
        if (i < op->cam->stereo_pairs.at(i)) {
          // Logic for sync stereo subscriber
          // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
          int cam_id0 = i;
          int cam_id1 = op->cam->stereo_pairs.at(i);
          // Create sync filter (they have unique pointers internally, so we have to use move logic here...)
          auto image_sub0 = std::make_shared<message_filters::Subscriber<Image>>(*nh, op->cam->topic.at(cam_id0), 1);
          auto image_sub1 = std::make_shared<message_filters::Subscriber<Image>>(*nh, op->cam->topic.at(cam_id1), 1);
          auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
          sync->registerCallback(boost::bind(&ROSSubscriber::callback_stereo_I, this, _1, _2, 0, 1));
          // Append to our vector of subscribers
          sync_cam.push_back(sync);
          sync_subs_cam.push_back(image_sub0);
          sync_subs_cam.push_back(image_sub1);
          PRINT2("subscribing to cam (stereo): %s\n", op->cam->topic.at(cam_id0).c_str());
          PRINT2("subscribing to cam (stereo): %s\n", op->cam->topic.at(cam_id1).c_str());
        }
      } else {
        // create MONO subscriber
        subs.push_back(nh->subscribe<Image>(op->cam->topic.at(i), 10, boost::bind(&ROSSubscriber::callback_monocular_I, this, _1, i)));
        PRINT2("subscribing to cam (mono): %s\n", op->cam->topic.at(i).c_str());
      }
    }
  }

  // Create wheel subscriber
  if (op->wheel->enabled) {
    subs.push_back(nh->subscribe(op->wheel->topic, 1000, &ROSSubscriber::callback_wheel, this));
    PRINT2("subscribing to wheel: %s\n", op->wheel->topic.c_str());
  }

  // Create gps subscriber
  if (op->gps->enabled) {
    for (int i = 0; i < op->gps->max_n; i++) {
      subs.push_back(nh->subscribe<NavSatFix>(op->gps->topic.at(i), 1000, boost::bind(&ROSSubscriber::callback_gnss, this, _1, i)));
      PRINT2("subscribing to GNSS: %s\n", op->gps->topic.at(i).c_str());
    }
  }

  // Create lidar subscriber
  if (op->lidar->enabled) {
    for (int i = 0; i < op->lidar->max_n; i++) {
      subs.push_back(nh->subscribe<PointCloud2>(op->lidar->topic.at(i), 2, boost::bind(&ROSSubscriber::callback_lidar, this, _1, i)));
      PRINT2("subscribing to LiDAR: %s\n", op->lidar->topic.at(i).c_str());
    }
  }
}

void ROSSubscriber::callback_inertial(const Imu::ConstPtr &msg) {
  // convert into correct format & send it to our system
  ov_core::ImuData imu = ROSHelper::Imu2Data(msg);
  if (sys->feed_measurement_imu(imu)) {
    pub->visualize();
  }
  pub->publish_imu();
  PRINT1(YELLOW "[SUB] IMU measurement: %.3f" RESET, imu.timestamp);
  PRINT1(YELLOW "|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
}

void ROSSubscriber::callback_monocular_I(const ImageConstPtr &msg, int cam_id) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  if (ROSHelper::Image2Data(msg, cam_id, cam, op->cam)) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images(cam_id);
    PRINT1(YELLOW "[SUB] MONO Cam measurement: %.3f|%d\n" RESET, msg->header.stamp.toSec(), cam_id);
  }
}

void ROSSubscriber::callback_monocular_C(const CompressedImageConstPtr &msg, int cam_id) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  if (ROSHelper::Image2Data(msg, cam_id, cam, op->cam)) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images(cam_id);
    PRINT1(YELLOW "[SUB] MONO Cam measurement: %.3f|%d\n" RESET, msg->header.stamp.toSec(), cam_id);
  }
}

void ROSSubscriber::callback_stereo_I(const ImageConstPtr &msg0, const ImageConstPtr &msg1, int cam_id0, int cam_id1) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  bool success0 = ROSHelper::Image2Data(msg0, cam_id0, cam, op->cam);
  bool success1 = ROSHelper::Image2Data(msg1, cam_id1, cam, op->cam);
  if (success0 && success1) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images({cam_id0, cam_id1});
    PRINT1(YELLOW "[SUB] STEREO Cam measurement: %.3f|%d|%d\n" RESET, msg0->header.stamp.toSec(), cam_id0, cam_id1);
  }
}

void ROSSubscriber::callback_stereo_C(const CompressedImageConstPtr &msg0, const CompressedImageConstPtr &msg1, int cam_id0, int cam_id1) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  bool success0 = ROSHelper::Image2Data(msg0, cam_id0, cam, op->cam);
  bool success1 = ROSHelper::Image2Data(msg1, cam_id1, cam, op->cam);
  if (success0 && success1) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images({cam_id0, cam_id1});
    PRINT1(YELLOW "[SUB] STEREO Cam measurement: %.3f|%d|%d\n" RESET, msg0->header.stamp.toSec(), cam_id0, cam_id1);
  }
}

void ROSSubscriber::callback_wheel(const JointStateConstPtr &msg) {
  // Return if the message contains other than wheel measurement info
  if (find(op->wheel->sub_topics.begin(), op->wheel->sub_topics.end(), msg->name.at(0)) == op->wheel->sub_topics.end())
    return;

  WheelData data = ROSHelper::JointState2Data(msg);
  sys->feed_measurement_wheel(data);
  PRINT1(YELLOW "[SUB] Wheel measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
}

void ROSSubscriber::callback_gnss(const NavSatFixConstPtr &msg, int gps_id) {
  // convert into correct format & send it to our system
  GPSData data = ROSHelper::NavSatFix2Data(msg, gps_id);
  // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
  data.noise(0) <= 0.0 || op->gps->overwrite_noise ? data.noise(0) = op->gps->noise : double();
  data.noise(1) <= 0.0 || op->gps->overwrite_noise ? data.noise(1) = op->gps->noise : double();
  data.noise(2) <= 0.0 || op->gps->overwrite_noise ? data.noise(2) = op->gps->noise * 2 : double();
  sys->feed_measurement_gps(data, true);
  pub->publish_gps(data, true);
  PRINT1(YELLOW "[SUB] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
  PRINT1(YELLOW "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
}

void ROSSubscriber::callback_lidar(const PointCloud2ConstPtr &msg, int lidar_id) {
  // convert into correct format & send it to our system
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data = ROSHelper::rosPC2pclPC(msg, lidar_id);
  sys->feed_measurement_lidar(data);
  pub->publish_lidar_cloud(data);
  PRINT1(YELLOW "[SUB] LiDAR measurement: %.3f|%d\n" RESET, (double)msg->header.stamp.toSec() / 1000, lidar_id);
}
