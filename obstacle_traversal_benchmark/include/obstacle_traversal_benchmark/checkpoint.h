#ifndef OBSTACLE_TRAVERSAL_BENCHMARK_CHECKPOINT_H
#define OBSTACLE_TRAVERSAL_BENCHMARK_CHECKPOINT_H

#include <Eigen/Eigen>

namespace obstacle_traversal_benchmark {

struct Checkpoint {
  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
};

}

#endif