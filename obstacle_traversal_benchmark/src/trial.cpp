#include <obstacle_traversal_benchmark/trial.h>

#include <obstacle_traversal_benchmark/util.h>
#include <fstream>

namespace obstacle_traversal_benchmark {

Trial::Trial(const ros::Time& start_time)
: start_time_(start_time) {}

bool Trial::saveToCsv(const std::string &csv_file_path) const {
  std::string imu_file_path = csv_file_path + "-imu.csv";
  if (!saveImuToCsv(imu_file_path)) {
    return false;
  }
  std::string stability_file_path = csv_file_path + "-stability.csv";
  if (!saveStabilityToCsv(stability_file_path)) {
    return false;
  }
  return true;
}

bool Trial::saveImuToCsv(const std::string &csv_file_path) const {
  if (!createParentDirectory(csv_file_path)) {
    return false;
  }
  if (imu_data_.empty()) {
    ROS_WARN_STREAM("Tried to save empty IMU data.");
    return true;
  }

  std::ofstream file;
  file.open(csv_file_path);

  file << "time,"
       << getVector3Labels("angular_velocity")
       << getVector3Labels("linear_acceleration")
       << std::endl;
  for (const auto& data_point: imu_data_) {
    ros::Duration duration_since_start = data_point.header.stamp - start_time_;
    file << duration_since_start.toSec() << ", ";
    file << vector3ToText(data_point.angular_velocity) << ", ";
    file << vector3ToText(data_point.linear_acceleration) << std::endl;
  }

  file.close();
  return true;
}

bool Trial::saveStabilityToCsv(const std::string &csv_file_path) const {
  if (!createParentDirectory(csv_file_path)) {
    return false;
  }
  if (stability_data_.empty()) {
    ROS_WARN_STREAM("Tried to save empty stability data.");
    return true;
  }

  std::ofstream file;
  file.open(csv_file_path);

  file << "time, "
       << getPoseLabels("robot_pose") << ", "
       << "stability"
       << std::endl;

  for (const auto& data_point: stability_data_) {
    ros::Duration duration_since_start = data_point.time - start_time_;
    file << poseToText(data_point.robot_pose) << ", ";
    file << data_point.estimated_stability << std::endl;
  }

  file.close();
  return true;
}

std::vector<sensor_msgs::Imu>& Trial::getImuData() {
  return imu_data_;
}

std::vector<StabilityDatapoint>& Trial::getStabilityData() {
  return stability_data_;
}

}