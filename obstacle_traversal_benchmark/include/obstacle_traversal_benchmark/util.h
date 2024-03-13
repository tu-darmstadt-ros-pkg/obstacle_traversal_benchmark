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

std::string vector3ToString(const Eigen::Vector3d& vec3);
std::string vector2ToString(const Eigen::Vector2d& vec2);


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

std::string poseToCsv(const hector_math::Pose<double>& pose);
std::string getPoseLabels(const std::string& pose_name);

std::string vector3ToCsv(const geometry_msgs::Vector3& vec);
std::string getVector3Labels(const std::string& base_name);

Eigen::Vector3d rotationToEulerAngles(const Eigen::Matrix3d &rot);

bool loadVector2FromXmlRpcValue(const XmlRpc::XmlRpcValue& vec2_list, Eigen::Vector2d& vec2, const std::string& parameter_ns);

// From https://stackoverflow.com/a/1968345
bool getLineIntersection(const Eigen::Vector2d& p0,
                         const Eigen::Vector2d& p1,
                         const Eigen::Vector2d& p2,
                         const Eigen::Vector2d& p3,
                         Eigen::Vector2d& intersection);




}

#endif  // OBSTACLE_TRAVERSAL_BENCHMARK_UTIL_H
