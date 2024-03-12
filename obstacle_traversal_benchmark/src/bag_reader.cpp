#include <obstacle_traversal_benchmark/bag_reader.h>

#include <sensor_msgs/JointState.h>

#include <obstacle_traversal_benchmark/util.h>

namespace obstacle_traversal_benchmark {

BagReader::BagReader(std::string bag_path, std::vector<std::string> joint_names, double time_resolution)
  : bag_path_(std::move(bag_path)), joint_names_(std::move(joint_names)), time_resolution_(time_resolution)
{}

bool BagReader::parse(std::vector<Trial> &trials, const std::vector<Checkpoint>& checkpoints) {
  tf_buffer_.clear();
  joint_position_map_.clear();
  missing_joint_states_ = std::set<std::string>(joint_names_.begin(),joint_names_.end());

  rosbag::Bag bag;
  try {
    bag.open(bag_path_, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  std::vector<std::string> topics{"/tf", "/tf_static", "/joint_states", "/imu/data"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (const rosbag::MessageInstance& m: view) {
    // Check if next checkpoint has been passed
    // TODO
    if (checkpoints.empty() && trials.empty()) {
      trials.emplace_back();
    }

    // Handle joint state msg
    updateJointPositionMap(m);
    // Handle tf message
    bool buffer_updated = updateTfBuffer(m);

    // Add Data
    if (!trials.empty()) {
      Trial& current_trial = trials.back();
      addIMUMessage(current_trial, m);
      // Add robot poses
      if (missing_joint_states_.empty()) {
        if (buffer_updated) {
          addPose(current_trial);
        }
      } else {
        ROS_WARN_STREAM("Trial has started but missing joint states: " << setToString(missing_joint_states_));
      }
    }
  }
  bag.close();
  return true;
}

bool BagReader::updateTfBuffer(const rosbag::MessageInstance &msg) {
  tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
  if (tf_msg) {
    bool is_static = msg.getTopic() == "/tf_static";
    for (const auto& transform_msg: tf_msg->transforms) {
      ROS_DEBUG_STREAM("Added transform " << transform_msg.header.frame_id << " -> " << transform_msg.child_frame_id << " at time "
                                          << (is_static ? "static" : std::to_string(transform_msg.header.stamp.toSec())));
      tf_buffer_.setTransform(transform_msg, "BagReader", is_static);
    }
    return true;
  }
  return false;
}

void BagReader::updateJointPositionMap(const rosbag::MessageInstance& msg)
{
  sensor_msgs::JointState::ConstPtr joint_state_msg = msg.instantiate<sensor_msgs::JointState>();
  if (joint_state_msg) {
    // Mark seen joints
    if (!missing_joint_states_.empty()) {
      for (const auto & joint_name : joint_state_msg->name) {
        auto it = missing_joint_states_.find(joint_name);
        if (it != missing_joint_states_.end()) {
          missing_joint_states_.erase(it);
        }
      }
    }
    // Update state
    for (unsigned int i = 0; i < joint_state_msg->name.size(); ++i) {
      joint_position_map_[joint_state_msg->name[i]] = joint_state_msg->position[i];
    }
  }
}

void BagReader::addIMUMessage(Trial& trial, const rosbag::MessageInstance& msg) {
  sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
  if (imu_msg) {
    trial.getImuData().push_back(*imu_msg);
  }
}

void BagReader::addPose(Trial& trial) {
  // Get current pose
  geometry_msgs::TransformStamped transform_msg;
  try {
    transform_msg = tf_buffer_.lookupTransform("world", "base_link",ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }

  // Check if we add it
  bool add_pose = false;
  if (trial.getStabilityData().empty()) {
    add_pose = true;
  } else {
    ros::Duration time_diff = transform_msg.header.stamp - trial.getStabilityData().back().time;
    if (time_diff.toSec() > time_resolution_) {
      add_pose = true;
    }
  }

  // Add it
  if (add_pose) {
    StabilityDatapoint s;
    s.time = transform_msg.header.stamp;
    s.robot_pose = transformMsgToHectorMath(transform_msg.transform);
    s.joint_positions = joint_position_map_;
    trial.getStabilityData().push_back(std::move(s));
  }
}

}