#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_OBSTACLE_TRAVERSAL_BENCHMARK_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_OBSTACLE_TRAVERSAL_BENCHMARK_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/pose_predictor.h>
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::shared_ptr<BagReader> bag_reader_;

  // Parameters
  std::string bag_file_path_;
  std::string result_folder_;
  double time_resolution_;
  std::vector<Checkpoint> checkpoints_;
};

}

#endif