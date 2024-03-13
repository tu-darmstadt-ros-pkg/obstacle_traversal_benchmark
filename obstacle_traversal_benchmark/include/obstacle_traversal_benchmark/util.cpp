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

std::string poseToCsv(const hector_math::Pose<double>& pose) {
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

std::string vector3ToCsv(const geometry_msgs::Vector3& vec) {
  std::stringstream ss;
  ss << vec.x << ", ";
  ss << vec.y << ", ";
  ss << vec.z;
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

bool loadVector2FromXmlRpcValue(const XmlRpc::XmlRpcValue& vec2_list, Eigen::Vector2d& vec2, const std::string& parameter_ns) {
  if (vec2_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Parameter '" << parameter_ns << "' is not an array.");
    return false;
  }
  if (vec2_list.size() != 2) {
    ROS_ERROR_STREAM("Parameter " << parameter_ns << "' is not a Vector2");
  }
  for (int j = 0; j < 2; ++j) {
    switch (vec2_list[j].getType()) {
      case XmlRpc::XmlRpcValue::TypeDouble:
        vec2[j] = static_cast<double>(vec2_list[j]);
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        vec2[j] = static_cast<int>(vec2_list[j]);
        break;
      default:
        ROS_ERROR_STREAM("Parameter '" << parameter_ns << "[" << j << "] is not a numeral.");
        return false;
    }
  }
  return true;
}
std::string vector3ToString(const Eigen::Vector3d& vec3) {
  return "[" + std::to_string(vec3.x()) + ", " + std::to_string(vec3.y()) + ", " + std::to_string(vec3.z()) + "]";
}
std::string vector2ToString(const Eigen::Vector2d& vec2) {
  return "[" + std::to_string(vec2.x()) + ", " + std::to_string(vec2.y()) + "]";
}

bool getLineIntersection(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                         const Eigen::Vector2d& p3, Eigen::Vector2d& intersection) {
  Eigen::Vector2d s1 = p1 - p0;
  Eigen::Vector2d s2= p3 - p2;

  double f = -s2.x() * s1.y() + s1.x() * s2.y();
  if (f < 0.001) { // Collinear or parallel
    return false;
  }
  double s = (-s1.y() * (p0.x() - p2.x()) + s1.x() * (p0.y() - p2.y())) / f;
  double t = ( s2.x() * (p0.y() - p2.y()) - s2.y() * (p0.x() - p2.x())) / f;

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
  {
    // Collision detected
    intersection = p0 + t * s1;
    return true;
  }

  return false; // No collision
}

}
