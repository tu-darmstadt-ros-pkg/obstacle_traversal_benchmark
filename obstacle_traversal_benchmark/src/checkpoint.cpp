#include <obstacle_traversal_benchmark/checkpoint.h>

#include <tf/transform_broadcaster.h>

namespace obstacle_traversal_benchmark {

visualization_msgs::Marker Checkpoint::getMarker() const {
  const double width = 0.02;
  const double height = 1.0;

  // Calculate the center point
  Eigen::Vector2d center = (p1 + p2) / 2.0;

  // Calculate the orientation (assuming on x-y plane, rotate about z-axis)
  double angle = atan2(p2.y() - p1.y(), p2.x() - p1.x());

  tf::Quaternion quat;
  quat.setRPY(0, 0, angle); // Roll, Pitch, Yaw

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "checkpoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = height / 2.0;
  tf::quaternionTFToMsg(quat, marker.pose.orientation);
  marker.scale.x = (p1 - p2).norm(); // Length of the box is the distance between p1 and p2
  marker.scale.y = width;
  marker.scale.z = height;
  marker.color.r = 0.5F;
  marker.color.g = 0.5F;
  marker.color.b = 0.5F;
  marker.color.a = 0.8;

  return marker;
}

}