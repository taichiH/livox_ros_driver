/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace livox_distortion_filter
{
using RotWithStamp = std::map<ros::Time, Eigen::Matrix3d>;
class LivoxDistortionFilter
{
public:
  LivoxDistortionFilter();

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};
  ros::Subscriber custom_cloud_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher undistort_cloud_pub_;
  ros::Publisher distort_cloud_pub_;

  float buffer_size_;
  float minimum_buffer_size_;
  std::deque<geometry_msgs::TransformStamped> relative_transform_buffer_;
  sensor_msgs::Imu prev_imu_data_;
  void pointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr & custom_msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg);
  RotWithStamp getRotationBuffer(const ros::Time & base_timestamp);
  Eigen::Matrix3d searchRotation(ros::Time point_stamp, const RotWithStamp & rotation_buffer);
};

}  // namespace livox_distortion_filter
