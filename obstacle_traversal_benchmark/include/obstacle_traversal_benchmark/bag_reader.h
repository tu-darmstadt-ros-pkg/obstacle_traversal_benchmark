#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_BAG_READER_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_BAG_READER_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>

#include <obstacle_traversal_benchmark/trial.h>
#include <obstacle_traversal_benchmark/checkpoint.h>

namespace obstacle_traversal_benchmark {
class BagReader {
public:
  BagReader(std::string bag_path, std::vector<std::string> joint_names, double time_resolution);
  bool parse(std::vector<Trial>& trials, const std::vector<Checkpoint>& checkpoints);
  void loadParametersFromNamespace(const ros::NodeHandle& nh);
private:
  bool updateTfBuffer(const rosbag::MessageInstance& msg);
  void updateJointPositionMap(const rosbag::MessageInstance& msg);
  void addIMUMessage(Trial& trial, const rosbag::MessageInstance& msg);
  void addState(Trial& trial);

  tf2_ros::Buffer tf_buffer_;
  std::unordered_map<std::string, double> joint_position_map_;
  std::set<std::string> missing_joint_states_;
  std::string bag_path_;
  std::vector<std::string> joint_names_;

  // Parameters
  double time_resolution_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  std::string joint_states_topic_;
  std::string imu_topic;
};
}

#endif