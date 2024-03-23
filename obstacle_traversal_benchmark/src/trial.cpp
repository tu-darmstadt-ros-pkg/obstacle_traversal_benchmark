#include <obstacle_traversal_benchmark/trial.h>

#include <obstacle_traversal_benchmark/util.h>
#include <fstream>
#include <eigen_conversions/eigen_msg.h>

namespace obstacle_traversal_benchmark {

bool Trial::saveToCsv(const std::string &csv_file_path) const {
  std::string imu_file_path = csv_file_path + "_imu.csv";
  if (!saveImuToCsv(imu_file_path)) {
    return false;
  }
  std::string stability_file_path = csv_file_path + "_stability.csv";
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
       << getVector3Labels("linear_acceleration") << "\n";
  for (const auto& data_point: imu_data_) {
    ros::Duration duration_since_start = data_point.header.stamp - start_time_;
    file << duration_since_start.toSec() << ", ";
    file << vector3ToCsv(data_point.angular_velocity) << ", ";
    file << vector3ToCsv(data_point.linear_acceleration) << '\n';
  }

  file.close();
  ROS_INFO_STREAM("Saved IMU data to " << csv_file_path);
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
       << "travelled_distance, "
       << getPoseLabels("robot_pose") << ", "
       << "stability\n";

  for (const auto& data_point: stability_data_) {
    ros::Duration duration_since_start = data_point.time - start_time_;
    file << duration_since_start.toSec() << ", ";
    file << data_point.travelled_distance << ", ";
    file << poseToCsv(data_point.robot_pose) << ", ";
    file << data_point.estimated_stability << '\n';
  }

  file.close();
  ROS_INFO_STREAM("Saved stability data to " << csv_file_path);
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

  // Compute travelled distance
  if (stability_data_.empty()) {
    s.travelled_distance = 0.0;
  } else {
    double distance = (robot_pose.translation() - stability_data_.back().robot_pose.translation()).norm();
    s.travelled_distance = stability_data_.back().travelled_distance + distance;
  }
  stability_data_.push_back(std::move(s));
}

nav_msgs::Path Trial::getPath() const {
  nav_msgs::Path path_msg;
  if (stability_data_.empty()) {
    return path_msg;
  }
  path_msg.header.stamp = stability_data_.front().time;
  path_msg.header.frame_id = "world";
  path_msg.poses.reserve(stability_data_.size());
  for (const auto& dp: stability_data_) {
    Eigen::Isometry3d robot_pose = dp.robot_pose.asTransform();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = dp.time;
    pose_msg.header.frame_id = "world";
    tf::poseEigenToMsg(robot_pose, pose_msg.pose);
    path_msg.poses.push_back(std::move(pose_msg));
  }
  return path_msg;
}

}