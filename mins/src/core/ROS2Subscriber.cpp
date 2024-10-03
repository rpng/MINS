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

#include "ROS2Subscriber.h"
#include "ROS2Helper.h"
#include "ROS2Publisher.h"
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

ROS2Subscriber::ROS2Subscriber(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<SystemManager> sys, std::shared_ptr<ROS2Publisher> pub) : node(node), sys(sys), pub(pub) {
  // Copy option for easier access
  op = sys->state->op;

  // Create imu subscriber (handle legacy ros param info)
  //  subs.push_back(node->create_subscription<Imu>(op->imu->topic, rclcpp::SensorDataQoS(), std::bind(&ROS2Subscriber::callback_inertial, this, std::placeholders::_1)));

  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(op->imu->topic, rclcpp::QoS(100), std::bind(&ROS2Subscriber::callback_inertial, this, std::placeholders::_1));
  PRINT1("subscribing to imu: %s\n", sub_imu->get_topic_name());

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
          if (op->cam->compressed.at(cam_id0)) {
            auto image_sub0 = std::make_shared<message_filters::Subscriber<CompressedImage>>(node, op->cam->topic.at(cam_id0));
            auto image_sub1 = std::make_shared<message_filters::Subscriber<CompressedImage>>(node, op->cam->topic.at(cam_id1));
            auto sync = std::make_shared<message_filters::Synchronizer<csync_pol>>(csync_pol(10), *image_sub0, *image_sub1);
            sync->registerCallback(std::bind(&ROS2Subscriber::callback_stereo_C, this, std::placeholders::_1, std::placeholders::_2, cam_id0, cam_id1));
            // Append to our vector of subscribers
            csync_cam.push_back(sync);
            cimage_subs.push_back(image_sub0);
            cimage_subs.push_back(image_sub1);
            PRINT2("subscribing to cam (stereo, compressed): %s\n", op->cam->topic.at(cam_id0).c_str());
            PRINT2("subscribing to cam (stereo, compressed): %s\n", op->cam->topic.at(cam_id1).c_str());

          } else {
            auto image_sub0 = std::make_shared<message_filters::Subscriber<Image>>(node, op->cam->topic.at(cam_id0));
            auto image_sub1 = std::make_shared<message_filters::Subscriber<Image>>(node, op->cam->topic.at(cam_id1));
            auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
            sync->registerCallback(std::bind(&ROS2Subscriber::callback_stereo_I, this, std::placeholders::_1, std::placeholders::_2, cam_id0, cam_id1));
            // Append to our vector of subscribers
            sync_cam.push_back(sync);
            image_subs.push_back(image_sub0);
            image_subs.push_back(image_sub1);
            PRINT2("subscribing to cam (stereo): %s\n", op->cam->topic.at(cam_id0).c_str());
            PRINT2("subscribing to cam (stereo): %s\n", op->cam->topic.at(cam_id1).c_str());
          }
        }
      } else {
        // create MONO subscriber
        if (op->cam->compressed.at(i)) {
          auto image_sub = std::make_shared<message_filters::Subscriber<CompressedImage>>(node, op->cam->topic.at(i));
          image_sub->registerCallback(std::bind(&ROS2Subscriber::callback_monocular_C, this, std::placeholders::_1, i));
          cimage_subs.push_back(image_sub);
          PRINT2("subscribing to cam (mono, compressed): %s\n", op->cam->topic.at(i).c_str());
        } else {
          auto image_sub = std::make_shared<message_filters::Subscriber<Image>>(node, op->cam->topic.at(i));
          image_sub->registerCallback(std::bind(&ROS2Subscriber::callback_monocular_I, this, std::placeholders::_1, i));
          image_subs.push_back(image_sub);
          PRINT2("subscribing to cam (mono): %s\n", op->cam->topic.at(i).c_str());
        }
      }
    }
  }

  // Create wheel subscriber
  if (op->wheel->enabled) {
      subs.push_back(node->create_subscription<JointState>(op->wheel->topic, rclcpp::SensorDataQoS(), std::bind(&ROS2Subscriber::callback_wheel, this, std::placeholders::_1)));
      PRINT2("subscribing to wheel: %s\n", op->wheel->topic.c_str());
  }

  // Create gps subscriber
  if (op->gps->enabled) {
    for (int i = 0; i < op->gps->max_n; i++) {
      subs.push_back(node->create_subscription<NavSatFix>(op->gps->topic.at(i), rclcpp::QoS(1000), [this, i](const NavSatFix::SharedPtr msg0) { this->callback_gnss(msg0, i); }));
      PRINT2("subscribing to GNSS: %s\n", op->gps->topic.at(i).c_str());
    }
  }

  // Create lidar subscriber
  if (op->lidar->enabled) {
    for (int i = 0; i < op->lidar->max_n; i++) {
      subs.push_back(node->create_subscription<PointCloud2>(op->lidar->topic.at(i), 2, [this, i](const PointCloud2::SharedPtr msg) { this->callback_lidar(msg, i); }));
      PRINT2("subscribing to LiDAR: %s\n", op->lidar->topic.at(i).c_str());
    }
  }

  reset_srv = node->create_service<std_srvs::srv::Empty>("/mins/reset", std::bind(&ROS2Subscriber::reset_service, this, std::placeholders::_1, std::placeholders::_2));
}

void ROS2Subscriber::callback_inertial(const Imu::SharedPtr msg) {
  // convert into correct format & send it to our system
  ov_core::ImuData imu = ROS2Helper::Imu2Data(msg);
  if (sys->feed_measurement_imu(imu)) {
    pub->visualize();
  }
  pub->publish_imu();
  PRINT1(YELLOW "[SUB] IMU measurement: %.3f" RESET, imu.timestamp);
  PRINT1(YELLOW "|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
}

void ROS2Subscriber::callback_monocular_I(const Image::ConstSharedPtr msg, int cam_id) {
  // convert into correct format & send it to our system
  //
  ov_core::CameraData cam;
  if (ROS2Helper::Image2Data(msg, cam_id, cam, op->cam)) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images(cam_id);
    PRINT1(YELLOW "[SUB] MONO Cam measurement: %.3f|%d\n" RESET, cam.timestamp, cam_id);
  }
}

void ROS2Subscriber::callback_monocular_C(const CompressedImage::ConstSharedPtr msg, int cam_id) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  if (ROS2Helper::Image2Data(msg, cam_id, cam, op->cam)) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images(cam_id);
    PRINT1(YELLOW "[SUB] MONO Cam measurement: %.3f|%d\n" RESET, cam.timestamp, cam_id);
  }
}

void ROS2Subscriber::callback_stereo_I(const Image::ConstSharedPtr msg0, const Image::ConstSharedPtr msg1, int cam_id0, int cam_id1) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  bool success0 = ROS2Helper::Image2Data(msg0, cam_id0, cam, op->cam);
  bool success1 = ROS2Helper::Image2Data(msg1, cam_id1, cam, op->cam);
  if (success0 && success1) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images({cam_id0, cam_id1});
    PRINT1(YELLOW "[SUB] STEREO Cam measurement: %.3f|%d|%d\n" RESET, cam.timestamp, cam_id0, cam_id1);
  } else {
    PRINT_ERROR("[SUB] Stereo Image Error!");
  }
}


void ROS2Subscriber::callback_stereo_C(const CompressedImage::ConstSharedPtr msg0, const CompressedImage::ConstSharedPtr msg1, int cam_id0, int cam_id1) {
  // convert into correct format & send it to our system
  ov_core::CameraData cam;
  bool success0 = ROS2Helper::Image2Data(msg0, cam_id0, cam, op->cam);
  bool success1 = ROS2Helper::Image2Data(msg1, cam_id1, cam, op->cam);
  if (success0 && success1) {
    sys->feed_measurement_camera(cam);
    pub->publish_cam_images({cam_id0, cam_id1});
    PRINT1(YELLOW "[SUB] STEREO Cam measurement: %.3f|%d|%d\n" RESET, rclcpp::Time(msg0->header.stamp).seconds(), cam_id0, cam_id1);
  }
}

void ROS2Subscriber::callback_wheel(const JointState::SharedPtr msg) {
  // Return if the message contains other than wheel measurement info
  if (find(op->wheel->sub_topics.begin(), op->wheel->sub_topics.end(), msg->name.at(0)) == op->wheel->sub_topics.end())
    return;

  WheelData data = ROS2Helper::JointState2Data(msg);
  sys->feed_measurement_wheel(data);
  PRINT1(YELLOW "[SUB] Wheel measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
}

void ROS2Subscriber::callback_gnss(const NavSatFix::SharedPtr msg, int gps_id) {
  // convert into correct format & send it to our system
  GPSData data = ROS2Helper::NavSatFix2Data(msg, gps_id);
  // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
  data.noise(0) <= 0.0 || op->gps->overwrite_noise ? data.noise(0) = op->gps->noise : double();
  data.noise(1) <= 0.0 || op->gps->overwrite_noise ? data.noise(1) = op->gps->noise : double();
  data.noise(2) <= 0.0 || op->gps->overwrite_noise ? data.noise(2) = op->gps->noise * 2 : double();
  sys->feed_measurement_gps(data, true);
  pub->publish_gps(data, true);
  PRINT1(YELLOW "[SUB] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
  PRINT1(YELLOW "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
}

void ROS2Subscriber::callback_lidar(const PointCloud2::SharedPtr msg, int lidar_id) {
  // convert into correct format & send it to our system
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data = ROS2Helper::rosPC2pclPC(msg, lidar_id);
  sys->feed_measurement_lidar(data);
  pub->publish_lidar_cloud(data);
  PRINT1(YELLOW "[SUB] LiDAR measurement: %.3f|%d\n" RESET, (double)msg->header.stamp.sec / 1000, lidar_id);
}

void ROS2Subscriber::reset_service(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  sys->init();
  pub->reset_paths();
}