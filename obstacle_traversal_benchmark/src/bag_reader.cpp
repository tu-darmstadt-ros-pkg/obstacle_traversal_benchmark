#include <obstacle_traversal_benchmark/bag_reader.h>

#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>

#include <obstacle_traversal_benchmark/util.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace obstacle_traversal_benchmark {

BagReader::BagReader(std::string bag_path, std::vector<std::string> joint_names, double time_resolution)
  : bag_path_(std::move(bag_path)),
    joint_names_(std::move(joint_names)),
    time_resolution_(time_resolution),
    tf_topic_("/tf"),
    tf_static_topic_("/tf_static"),
    joint_states_topic_("/joint_states"),
    imu_topic("/imu/data")
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

  std::vector<std::string> topics{tf_topic_, tf_static_topic_, joint_states_topic_, imu_topic};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  bool last_pose_set = false;
  size_t next_checkpoint_index = 0;
  Eigen::Isometry3d last_pose;
  ros::Time last_pose_stamp;
  std::vector<Checkpoint> checkpoints_temp = checkpoints;
  ros::Time last_checkpoint_crossed;
  bool first_checkpoint_crossed = false;
  bool wait_for_next_trial = !checkpoints_temp.empty();
  for (const rosbag::MessageInstance& m: view) {
    // Handle joint state msg
    updateJointPositionMap(m);
    // Handle tf message
    bool buffer_updated = updateTfBuffer(m);

    if (checkpoints_temp.empty()) {
      if (trials.empty()) {
        ROS_INFO_STREAM("No checkpoints configured. Creating single trial.");
        trials.emplace_back();
      }
    } else {
      // Check if next checkpoint has been passed
      geometry_msgs::TransformStamped transform_msg;
      try {
        transform_msg = tf_buffer_.lookupTransform("world", "base_link",ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        continue;
      }
      Eigen::Isometry3d current_pose;
      tf::transformMsgToEigen(transform_msg.transform, current_pose);

      if (last_pose_set) {
        ros::Duration time_diff = transform_msg.header.stamp - last_pose_stamp;
        if (time_diff.toSec() > time_resolution_) {
          const Checkpoint& next_checkpoint = checkpoints_temp[next_checkpoint_index];
          Eigen::Vector2d previous_position = last_pose.translation().block<2, 1>(0, 0);
          Eigen::Vector2d current_position = current_pose.translation().block<2, 1>(0, 0);
          Eigen::Vector2d intersection;
          bool intersect = getLineIntersection(next_checkpoint.p1, next_checkpoint.p2, previous_position, current_position, intersection);
          bool checkpoint_crossed = intersect && (!first_checkpoint_crossed || (m.getTime() - last_checkpoint_crossed).toSec() > 2.0);
          if (checkpoint_crossed) {
            first_checkpoint_crossed = true;
            last_checkpoint_crossed = m.getTime();

            next_checkpoint_index++;
            ros::Duration bag_duration = m.getTime() - view.getBeginTime();
            boost::posix_time::ptime posix_time = m.getTime().toBoost();
            std::string time_date_str = boost::posix_time::to_iso_extended_string(posix_time);

            std::string full_stamp = "[secs: " + std::to_string(m.getTime().sec) + ", nsecs: " + std::to_string(m.getTime().nsec) + "]";


            bool start_new_trial;
            if (checkpoints_temp.size() > 1) {
              // Do not start new trial after last checkpoint
              start_new_trial = next_checkpoint_index != checkpoints_temp.size();
              if (!start_new_trial) {
                std::reverse(checkpoints_temp.begin(), checkpoints_temp.end());
              }
            } else {
              // Start new trial every other pass of single checkpoint
              start_new_trial = wait_for_next_trial;
            }
            next_checkpoint_index = next_checkpoint_index % checkpoints_temp.size();

            if (start_new_trial) {
//              ROS_INFO_STREAM("Started trial " << trials.size() << " at " << bag_duration.toSec() << "s since bag start.");
              ROS_INFO_STREAM("Started trial " << trials.size() << " at " << full_stamp << ".");
              trials.emplace_back();
              wait_for_next_trial = false;
            } else {
//              ROS_INFO_STREAM("Finished trial " << trials.size() - 1<< " at " << bag_duration.toSec() << "s since bag start.\n");
              ROS_INFO_STREAM("Finished trial " << trials.size() - 1<< " at " << full_stamp << ".");
              wait_for_next_trial = true;
              next_checkpoint_index = 0;
              const double min_trial_time = 3.0;
              if (trials.back().getDuration().toSec() < min_trial_time) {
                ROS_WARN_STREAM("Last trial shorter than " << min_trial_time << "s. Removing trial.");
                trials.pop_back();
              }
            }

          }
          last_pose = current_pose;
          last_pose_stamp = transform_msg.header.stamp;
        }
      } else {
        last_pose = current_pose;
        last_pose_stamp = transform_msg.header.stamp;
        last_pose_set = true;
      }




    }

    // Add Data
    if (!trials.empty() && !wait_for_next_trial) {
      Trial& current_trial = trials.back();
      addIMUMessage(current_trial, m);
      // Add robot poses
      if (missing_joint_states_.empty()) {
        if (buffer_updated) {
          addState(current_trial);
        }
      } else {
        ROS_WARN_STREAM("Trial has started but missing joint states: " << setToString(missing_joint_states_));
      }
    }
  }
  if (!checkpoints_temp.empty() && !trials.empty() && !wait_for_next_trial) {
    // Ditch the last incomplete trial
    trials.pop_back();
  }
  bag.close();
  return true;
}

bool BagReader::updateTfBuffer(const rosbag::MessageInstance &msg) {
  tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
  if (tf_msg) {
    bool is_static = msg.getTopic() == tf_static_topic_;
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
    trial.addImuData(*imu_msg);
  }
}

void BagReader::addState(Trial& trial) {
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
    trial.addStateData(transform_msg.header.stamp,
                       transformMsgToHectorMath(transform_msg.transform),
                       joint_position_map_);
  }
}
void BagReader::loadParametersFromNamespace(const ros::NodeHandle &nh) {
  nh.getParam("tf_topic", tf_topic_);
  nh.getParam("tf_static_topic", tf_static_topic_);
  nh.getParam("joint_states_topic", joint_states_topic_);
  nh.getParam("imu_topic", imu_topic);
}

}