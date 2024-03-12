#include <ros/ros.h>

#include <obstacle_traversal_benchmark/obstacle_traversal_benchmark.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_traversal_benchmark");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  obstacle_traversal_benchmark::ObstacleTraversalBenchmark benchmark(nh, pnh);
  benchmark.runEvaluation();

  ros::spin();
  return 0;
}