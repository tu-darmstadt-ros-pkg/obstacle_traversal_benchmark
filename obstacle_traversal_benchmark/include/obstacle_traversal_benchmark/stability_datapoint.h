#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_STABILITY_DATAPOINT_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_STABILITY_DATAPOINT_H

#include <ros/ros.h>
#include <hector_pose_prediction_interface/types.h>
#include <obstacle_traversal_benchmark/types.h>

namespace obstacle_traversal_benchmark {
struct StabilityDatapoint {
  ros::Time time;
  hector_math::Pose<double> robot_pose;
  double travelled_distance;
  JointPositionMap joint_positions;
  double estimated_stability;
};
}

#endif