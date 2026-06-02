#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");

int main(int argc, char** argv)
{
  // Initialize node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("tsrrtc_example", node_options);

  // Initialize single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Load in robot model
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  // Set up planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  // Get robot model (with accompanying kinematics)
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Lock scene state to get latest robot state information
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  // Get current joint state (useful for keeping track of)
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");

  // This is where we will specify our own custom planner
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
    
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "move_group_tutorial", psm);
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // Display example text
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "TSRRT-Connect Example", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the example");

  // Pose Goal
  planning_interface::MotionPlanRequest req;
  req.pipeline_id = "ompl"; // SWAP THIS WITH OUR CUSTOM VERSION
  req.planner_id = "RRTConnectkConfigDefault"; // SWAP THIS WITH OUR CUSTOM VERSION
  req.allowed_planning_time = 1.0;
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.1);
  std::vector<double> tolerance_angle(3, 0.1);

  req.group_name = "panda_arm";
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    /* Check that the planning was successful */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  
  // Display resultant trajectory
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
      node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  
  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  /*
  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, set up a joint space goal
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::msg::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  
  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see two planned trajectories in series
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Using a Planning Request Adapter
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // A planning request adapter allows us to specify a series of operations that
  // should happen either before planning takes place or after the planning
  // has been done on the resultant path

  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, set one of the joints slightly outside its upper limit
  const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
  const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
  robot_state->setJointPositions(joint_model, tmp_values);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }
  
  RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  /* Now you should see three planned trajectories in series*/
  display_publisher->publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the example");

  RCLCPP_INFO(LOGGER, "Done");

  rclcpp::shutdown();
  return 0;
}