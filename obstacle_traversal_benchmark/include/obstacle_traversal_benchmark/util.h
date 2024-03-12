#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_UTIL_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_UTIL_H

#include <hector_math/types/pose.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>

namespace obstacle_traversal_benchmark {

bool createParentDirectory(const std::string& file_path);

template <typename T>
std::string setToString(const std::set<T>& set) {
  std::stringstream ss;
  ss << "[";
  for (auto entry: set) {
    ss << entry << ",";
  }
  ss << "]";
  return ss.str();
}

template <typename T>
bool loadMandatoryParameter(const ros::NodeHandle& nh, const std::string& key, T& val) {
  if (!nh.getParam(key, val)) {
    ROS_ERROR_STREAM("Failed to load mandatory parameter '" << nh.getNamespace() << "/" << key << "'.");
    return false;
  }
  return true;
}

hector_math::Pose<double> poseMsgToHectorMath(const geometry_msgs::Pose& pose_msg);
hector_math::Pose<double> transformMsgToHectorMath(const geometry_msgs::Transform& transform_msg);


void updateJointPositionMap(const sensor_msgs::JointStateConstPtr& joint_state_msg,
                            std::unordered_map<std::string, double>& joint_state_map,
                            std::set<std::string>& missing_joint_states);

Eigen::Vector3d rotationToEulerAngles(const Eigen::Matrix3d &rot);

}

#endif  // OBSTACLE_TRAVERSAL_BENCHMARK_UTIL_H
