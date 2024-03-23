#include <obstacle_traversal_benchmark/obstacle_traversal_benchmark.h>

#include <obstacle_traversal_benchmark/util.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <filesystem>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <hector_pose_prediction_ros/visualization.h>

namespace obstacle_traversal_benchmark {

ObstacleTraversalBenchmark::ObstacleTraversalBenchmark(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
 : nh_(nh), pnh_(pnh), time_resolution_(0.1)
{
  loadParameters(pnh);
  ros::NodeHandle pose_prediction_nh(pnh, "sdf_contact_estimation");
  pose_predictor_ = createPosePredictor(pose_prediction_nh);
  bag_reader_ = std::make_shared<BagReader>(bag_file_path_, pose_predictor_->robotModel()->jointNames(), time_resolution_);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  robot_model_ = robot_model_loader.getModel();
  if (!robot_model_){
    ROS_ERROR("Could not load robot model!");
  } else {
    robot_state_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("robot_state", 10, false);
  }

  checkpoint_vis_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("checkpoints", 10, true);
  publishCheckpoints();
}

void ObstacleTraversalBenchmark::runEvaluation() {
  std::vector<Trial> trials;
  if (!bag_reader_->parse(trials, checkpoints_)) {
    return;
  }

  for (size_t i = 0; i < trials.size(); ++i) {
    Trial& trial = trials[i];
    estimateStability(trial);

    std::filesystem::path bag_path(bag_file_path_);
    std::string base_path = result_folder_ + "/" + bag_path.stem().string() + "_trial-" + std::to_string(i);
    trial.saveToCsv(base_path);
  }

  publishPaths(trials);
}

bool ObstacleTraversalBenchmark::loadParameters(const ros::NodeHandle &nh) {
  if (!loadMandatoryParameter(nh, "bag_file_path", bag_file_path_)) {
    return false;
  }
  if (!loadMandatoryParameter(nh, "result_folder", result_folder_)) {
    return false;
  }
  time_resolution_ = nh.param<double>("time_resolution", 1.0);
  visualization_wait_time_ = nh.param<double>("visualization_wait_time", 0.0);

  XmlRpc::XmlRpcValue checkpoints_list;
  if (nh.getParam("checkpoints", checkpoints_list)) {
    if (checkpoints_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/checkpoints is not an array.");
      return false;
    }
    for (int i = 0; i < checkpoints_list.size(); ++i) {
      XmlRpc::XmlRpcValue checkpoint_dict = checkpoints_list[i];
      if (checkpoint_dict.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/checkpoints[" << i << "] is not a struct.");
        continue;
      }
      Checkpoint checkpoint;
      if (checkpoint_dict.hasMember("p1")) {
        if (!loadVector2FromXmlRpcValue(checkpoint_dict["p1"], checkpoint.p1,
                                        nh.getNamespace() + "/checkpoints[" + std::to_string(i) + "]/p1")) {
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/checkpoints[" << i << "]/p1 is missing.");
        continue;
      }
      if (checkpoint_dict.hasMember("p2")) {
        if (!loadVector2FromXmlRpcValue(checkpoint_dict["p2"], checkpoint.p2,
                                        nh.getNamespace() + "/checkpoints[" + std::to_string(i) + "]/p2")) {
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/checkpoints[" << i << "]/p2 is missing.");
        continue;
      }
      ROS_INFO_STREAM("Loaded checkpoint " << checkpoints_.size() << ": " << vector2ToString(checkpoint.p1) << " --> " << vector2ToString(checkpoint.p2));
      checkpoints_.push_back(std::move(checkpoint));
    }
  }

  return true;
}

hector_pose_prediction_interface::PosePredictor<double>::Ptr
ObstacleTraversalBenchmark::createPosePredictor(const ros::NodeHandle &nh) {
  // Map
  ros::NodeHandle sdf_model_nh(nh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);
  // Robot model
  ros::NodeHandle shape_model_nh(nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  // Predictor
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(nh, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(false);
  return std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
}

void ObstacleTraversalBenchmark::estimateStability(Trial &trial) {
  for (auto& data_point: trial.getStabilityData()) {
    pose_predictor_->robotModel()->updateJointPositions(data_point.joint_positions);
    hector_pose_prediction_interface::SupportPolygon<double> estimated_support_polygon;
    if (pose_predictor_->estimateSupportPolygon(data_point.robot_pose, estimated_support_polygon)) {
      Eigen::Vector3d com_base = pose_predictor_->robotModel()->centerOfMass();
      Eigen::Vector3d com_world = data_point.robot_pose * com_base;
      Eigen::Vector3d gravity(0, 0, -9.81);
      data_point.estimated_stability = hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasureValue(estimated_support_polygon.contact_hull_points, estimated_support_polygon.edge_stabilities, com_world, gravity);
    } else {
      data_point.estimated_stability = std::nan("");
    }
    publishRobotState(data_point.robot_pose, data_point.joint_positions, estimated_support_polygon);
    ros::Duration(visualization_wait_time_).sleep();
  }
}
void ObstacleTraversalBenchmark::publishRobotState(
    const hector_math::Pose<double> &pose, const JointPositionMap &joint_positions,
    const hector_pose_prediction_interface::SupportPolygon<double> &support_polygon)
{
  if (!robot_model_) {
    return;
  }
  deleteAllMarkers(robot_state_pub_);
  robot_state::RobotState state(robot_model_);
  state.setVariablePositions(std::map<std::string, double>(joint_positions.begin(), joint_positions.end()));
  state.setJointPositions("world_virtual_joint", pose.asTransform());

  std_msgs::ColorRGBA color;
  color.r = 0;
  color.g = 1;
  color.b = 0;
  color.a = 0.7;
  visualization_msgs::MarkerArray array;
  state.getRobotMarkers(array, robot_model_->getLinkModelNames(), color, "robot_state", ros::Duration(0));

  hector_pose_prediction_interface::visualization::addSupportPolygonToMarkerArray(array, support_polygon, "world");
  robot_state_pub_.publish(array);
}

void ObstacleTraversalBenchmark::publishPaths(const std::vector<Trial> &trials) {
  for (size_t i = 0; i < trials.size(); ++i) {
    const auto& trial = trials[i];
    path_pubs_.push_back(pnh_.advertise<nav_msgs::Path>("path/trial_" + std::to_string(i), 10, true));
    path_pubs_.back().publish(trial.getPath());
  }
}

void ObstacleTraversalBenchmark::publishCheckpoints() {
  visualization_msgs::MarkerArray array;
  for (const auto& checkpoint: checkpoints_) {
    array.markers.push_back(checkpoint.getMarker());
  }
  fixMarkerIds(array);
  checkpoint_vis_pub_.publish(array);
}

}