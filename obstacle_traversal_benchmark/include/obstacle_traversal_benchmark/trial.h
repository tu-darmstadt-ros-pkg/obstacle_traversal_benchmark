#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_TRIAL_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_TRIAL_H

#include <obstacle_traversal_benchmark/imu_datapoint.h>
#include <sensor_msgs/Imu.h>
#include <obstacle_traversal_benchmark/stability_datapoint.h>

namespace obstacle_traversal_benchmark {

class Trial {
public:
  bool saveToCsv(const std::string& csv_file_path) const;
  bool saveImuToCsv(const std::string& csv_file_path) const;
  bool saveStabilityToCsv(const std::string& csv_file_path) const;

  void addImuData(const sensor_msgs::Imu& imu_msg);

  void addStateData(const ros::Time& time,
                    const hector_math::Pose<double>& robot_pose,
                    const JointPositionMap& joint_positions);

  std::vector<sensor_msgs::Imu>& getImuData();
  std::vector<StabilityDatapoint>& getStabilityData();
private:
  ros::Time start_time_;
  std::vector<sensor_msgs::Imu> imu_data_;
  std::vector<StabilityDatapoint> stability_data_;
};

}

#endif