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

#include "livox_distortion_filter/livox_distortion_filter.hpp"

namespace livox_distortion_filter
{
LivoxDistortionFilter::LivoxDistortionFilter()
{
  custom_cloud_sub_ =
    pnh_.subscribe("input/custom_cloud", 1, &LivoxDistortionFilter::pointCloudCallback, this);
  imu_sub_ = pnh_.subscribe("input/imu", 1, &LivoxDistortionFilter::imuCallback, this);
  pnh_.param<float>("buffer_size", buffer_size_, 1000);
  pnh_.param<float>("minimum_buffer_size", minimum_buffer_size_, 200);
  undistort_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/undistort", 1, true);
  distort_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/distort", 1, true);
}

void LivoxDistortionFilter::imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg)
{
  if (relative_transform_buffer_.size() > buffer_size_) {
    relative_transform_buffer_.pop_front();
  }

  const double dt = (imu_msg->header.stamp - prev_imu_data_.header.stamp).toSec();
  const double roll = imu_msg->angular_velocity.x * dt;
  const double pitch = imu_msg->angular_velocity.y * dt;
  const double yaw = imu_msg->angular_velocity.z * dt;
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  geometry_msgs::TransformStamped relative_transform;
  relative_transform.transform.translation.x = 0.0;
  relative_transform.transform.translation.y = 0.0;
  relative_transform.transform.translation.z = 0.0;
  relative_transform.transform.rotation.x = q.x();
  relative_transform.transform.rotation.y = q.y();
  relative_transform.transform.rotation.z = q.z();
  relative_transform.transform.rotation.w = q.w();
  relative_transform.header = imu_msg->header;
  relative_transform_buffer_.push_back(relative_transform);
}

RotWithStamp LivoxDistortionFilter::getRotationBuffer(const ros::Time & base_timestamp)
{
  size_t start_idx = 0;
  // search imu timestamp index closest to input pointcloud timestamp start idx
  for (int i = relative_transform_buffer_.size() - 1; i >= 0; --i) {
    const auto imu_stamp = relative_transform_buffer_.at(i).header.stamp;
    if ((imu_stamp - base_timestamp).toSec() < 0) {
      start_idx = i;
      break;
    }
  }

  RotWithStamp rotation_buffer;
  Eigen::Matrix3d prev_rot;
  for (int i = start_idx; i < relative_transform_buffer_.size(); ++i) {
    const geometry_msgs::TransformStamped transform = relative_transform_buffer_.at(i);
    if (rotation_buffer.empty()) {
      prev_rot = Eigen::Matrix3d::Identity();
      continue;
    }
    Eigen::Quaterniond quat;
    tf2::fromMsg(transform.transform.rotation, quat);
    Eigen::Matrix3d current_rot = quat.toRotationMatrix();
    const auto base2point_rot = prev_rot * current_rot;
    rotation_buffer[transform.header.stamp] = base2point_rot;
    prev_rot = current_rot;
  }
  return rotation_buffer;
}

Eigen::Matrix3d LivoxDistortionFilter::searchRotation(
  ros::Time point_stamp, const RotWithStamp & rotation_buffer)
{
  Eigen::Matrix3d target_rot;
  double prev_timediff = std::numeric_limits<double>::max();
  for (auto itr = rotation_buffer.begin(); itr != rotation_buffer.end(); ++itr) {
    const double time_diff = (itr->first - point_stamp).toSec();
    if (time_diff < prev_timediff) {
      target_rot = itr->second;
      prev_timediff = time_diff;
    }
  }
  return target_rot;
}

void LivoxDistortionFilter::pointCloudCallback(
  const livox_ros_driver::CustomMsg::ConstPtr & custom_msg)
{
  std::cerr << "relative_transform_buffer_.size(): " << relative_transform_buffer_.size()
            << std::endl;
  if (relative_transform_buffer_.size() < minimum_buffer_size_) {
    return;
  }
  ros::Time base_timestamp;
  base_timestamp.fromNSec(custom_msg->timebase);

  // get transforms trajectory from input pointcloud start timestamp
  RotWithStamp rotation_buffer = getRotationBuffer(base_timestamp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr undistort_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr distort_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < custom_msg->point_num; ++i) {
    const auto p = custom_msg->points[i];
    /*     double offset_sec = p.offset_time * std::pow(1000, 3); */
    ros::Time point_stamp;
    point_stamp.fromNSec(custom_msg->timebase + p.offset_time);
    {
      const auto target_rot = searchRotation(point_stamp, rotation_buffer);
      Eigen::Vector3d undistort_point = target_rot.inverse() * Eigen::Vector3d(p.x, p.y, p.z);
      pcl::PointXYZ pcl_undistort_point(
        undistort_point.x(), undistort_point.y(), undistort_point.z());
      undistort_cloud->points.push_back(pcl_undistort_point);
    }
    {
      pcl::PointXYZ pcl_distort_point(p.x, p.y, p.z);
      distort_cloud->points.push_back(pcl_distort_point);
    }
  }
  sensor_msgs::PointCloud2 undistort_cloud_msg;
  undistort_cloud_msg.header = custom_msg->header;
  pcl::fromROSMsg(undistort_cloud_msg, *undistort_cloud);
  undistort_cloud_pub_.publish(undistort_cloud_msg);

  sensor_msgs::PointCloud2 distort_cloud_msg;
  distort_cloud_msg.header = custom_msg->header;
  pcl::fromROSMsg(distort_cloud_msg, *distort_cloud);
  distort_cloud_pub_.publish(distort_cloud_msg);
}
}  // namespace livox_distortion_filter
