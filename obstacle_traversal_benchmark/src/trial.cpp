#include <obstacle_traversal_benchmark/trial.h>

namespace obstacle_traversal_benchmark {

bool Trial::saveToCsv(const std::string &csv_file_path) const {
  return false;
}
bool Trial::saveImuToCsv(const std::string &csv_file_path) const {
  return false;
}
bool Trial::saveStabilityToCsv(const std::string &csv_file_path) const {
  return false;
}

std::vector<sensor_msgs::Imu>& Trial::getImuData() {
  return imu_data_;
}
std::vector<StabilityDatapoint>& Trial::getStabilityData() {
  return stability_data_;
}

}