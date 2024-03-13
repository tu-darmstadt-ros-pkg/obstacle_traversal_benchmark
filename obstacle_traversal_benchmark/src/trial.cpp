#include <obstacle_traversal_benchmark/trial.h>

#include <obstacle_traversal_benchmark/util.h>
#include <fstream>

namespace obstacle_traversal_benchmark {

bool Trial::saveToCsv(const std::string &csv_file_path) const {
  std::string imu_file_path = csv_file_path + "-imu.csv";
  if (!saveImuToCsv(imu_file_path)) {
    return false;
  }
  std::string stability_file_path = csv_file_path + "-stability.csv";
  return saveStabilityToCsv(stability_file_path);
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
       << getVector3Labels("angular_velocity") << ", "
       << getVector3Labels("linear_acceleration") << ", "
       << '\n';
  for (const auto& data_point: imu_data_) {
    ros::Duration duration_since_start = data_point.header.stamp - start_time_;
    file << duration_since_start.toSec() << ", ";
    file << vector3ToCsv(data_point.angular_velocity) << ", ";
    file << vector3ToCsv(data_point.linear_acceleration) << '\n';
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
       << "stability, "
       << '\n';

  for (const auto& data_point: stability_data_) {
    ros::Duration duration_since_start = data_point.time - start_time_;
    file << duration_since_start.toSec() << ", ";
    file << poseToCsv(data_point.robot_pose) << ", ";
    file << data_point.estimated_stability << '\n';
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

void Trial::addImuData(const sensor_msgs::Imu& imu_msg) {
  if (imu_data_.empty() && stability_data_.empty()) {
    start_time_ = imu_msg.header.stamp;
  }
  imu_data_.push_back(imu_msg);
}

void Trial::addStateData(const ros::Time& time, const hector_math::Pose<double>& robot_pose,
                         const JointPositionMap& joint_positions) {
  if (imu_data_.empty() && stability_data_.empty()) {
    start_time_ = time;
  }
  StabilityDatapoint s;
  s.time = time;
  s.robot_pose = robot_pose;
  s.joint_positions = joint_positions;
  stability_data_.push_back(std::move(s));
}

}