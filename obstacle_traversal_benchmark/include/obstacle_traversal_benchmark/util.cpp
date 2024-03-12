#include "util.h"

#include <filesystem>
#include <ros/console.h>

namespace obstacle_traversal_benchmark {

hector_math::Pose<double> poseMsgToHectorMath(const geometry_msgs::Pose& pose_msg) {
  return {pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
           pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y,
           pose_msg.orientation.z};
}

hector_math::Pose<double> transformMsgToHectorMath(const geometry_msgs::Transform& transform_msg) {
  return {transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z,
           transform_msg.rotation.w, transform_msg.rotation.x, transform_msg.rotation.y,
           transform_msg.rotation.z};
}

bool createParentDirectory(const std::string& file_path) {
  std::filesystem::path path(file_path);
  std::filesystem::path parent_directory = path.parent_path();
  try {
    std::filesystem::create_directories(parent_directory);
  } catch (const std::filesystem::filesystem_error& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  catch (const std::bad_alloc& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

std::string poseToText(const hector_math::Pose<double>& pose) {
  std::stringstream ss;
  ss << pose.translation().x() << ", ";
  ss << pose.translation().y() << ", ";
  ss << pose.translation().z() << ", ";
  Eigen::Vector3d input_rpy = rotationToEulerAngles(pose.asTransform().linear());
  ss << input_rpy(0) << ", ";
  ss << input_rpy(1) << ", ";
  ss << input_rpy(2);
  return ss.str();
}

std::string getPoseLabels(const std::string& pose_name) {
  std::stringstream ss;
  ss << pose_name << "_position_x,";
  ss << pose_name << "_position_y,";
  ss << pose_name << "_position_z,";
  ss << pose_name << "_orientation_roll,";
  ss << pose_name << "_orientation_pitch,";
  ss << pose_name << "_orientation_yaw";
  return ss.str();
}

std::string vector3ToText(const geometry_msgs::Vector3& vec) {
  std::stringstream ss;
  ss << vec.x << ", ";
  ss << vec.y << ", ";
  ss << vec.z << ", ";
  return ss.str();
}

std::string getVector3Labels(const std::string& base_name) {
  std::stringstream ss;
  ss << base_name << "_x,";
  ss << base_name << "_y,";
  ss << base_name << "_z";
  return ss.str();
}

Eigen::Vector3d rotationToEulerAngles(const Eigen::Matrix3d &rot) {
  double epsilon = 1e-12;
  double roll, pitch, yaw;
  pitch = std::atan2(-rot.data()[2], sqrt(rot.data()[0]*rot.data()[0] + rot.data()[1]*rot.data()[1]));
  if (std::abs(pitch) > (M_PI_2-epsilon)) {
    yaw = std::atan2(-rot.data()[3], rot.data()[4]);
    roll = 0.0;
  } else {
    roll = std::atan2(rot.data()[5], rot.data()[8]);
    yaw = std::atan2(rot.data()[1], rot.data()[0]);
  }
  return {roll, pitch, yaw};
}

}
