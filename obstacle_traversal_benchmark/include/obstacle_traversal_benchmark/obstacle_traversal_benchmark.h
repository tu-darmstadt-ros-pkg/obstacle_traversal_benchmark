#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_OBSTACLE_TRAVERSAL_BENCHMARK_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_OBSTACLE_TRAVERSAL_BENCHMARK_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/pose_predictor.h>
#include <moveit/robot_model/robot_model.h>
#include <visualization_msgs/MarkerArray.h>

#include <obstacle_traversal_benchmark/bag_reader.h>

namespace obstacle_traversal_benchmark {

class ObstacleTraversalBenchmark {
public:
  ObstacleTraversalBenchmark(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void runEvaluation();
private:
  bool loadParameters(const ros::NodeHandle& nh);
  hector_pose_prediction_interface::PosePredictor<double>::Ptr createPosePredictor(const ros::NodeHandle& nh);
  void estimateStability(Trial& trial);

  void publishRobotState(const hector_math::Pose<double>& pose,
                         const JointPositionMap& joint_positions,
                         const hector_pose_prediction_interface::SupportPolygon<double>& support_polygon);
  void publishPaths(const std::vector<Trial>& trials);
  void publishCheckpoints();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::shared_ptr<BagReader> bag_reader_;

  // Parameters
  std::string bag_file_path_;
  std::string result_folder_;
  double time_resolution_;
  std::vector<Checkpoint> checkpoints_;

  // State publishing
  double visualization_wait_time_;
  robot_model::RobotModelPtr robot_model_;
  ros::Publisher robot_state_pub_;
  std::vector<ros::Publisher> path_pubs_;
  ros::Publisher checkpoint_vis_pub_;
};

}

#endif