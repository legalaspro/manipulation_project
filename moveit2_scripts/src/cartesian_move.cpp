#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class CartesianMove {
public:
  CartesianMove(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Cartesian Move...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);

    // get initial state of robot
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Cartesian Move");
  }

  ~CartesianMove() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Cartesian Move");
  }

  void cartesian_move_to_target(double x, double y, double z) {
    // Get current pose
    geometry_msgs::msg::Pose current_pose =
        move_group_robot_->getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [%.3f, %.3f, %.3f]",
                current_pose.position.x, current_pose.position.y,
                current_pose.position.z);

    // Build target pose - use provided values
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    RCLCPP_INFO(LOGGER, "Target pose: [%.3f, %.3f, %.3f]",
                target_pose.position.x, target_pose.position.y,
                target_pose.position.z);

    // Check if we need to do sequential movement (Z first, then XY)
    bool move_z = (std::abs(z - current_pose.position.z) > 1e-6);
    bool move_xy = (std::abs(x - current_pose.position.x) > 1e-6 ||
                    std::abs(y - current_pose.position.y) > 1e-6);

    if (move_z && move_xy) {
      // Sequential movement: Z first, then XY
      RCLCPP_INFO(LOGGER, "Sequential movement detected: Z first, then XY");

      // Step 1: Move Z only
      RCLCPP_INFO(LOGGER, "Step 1: Moving Z axis to %.3f", z);
      geometry_msgs::msg::Pose z_pose = current_pose;
      z_pose.position.z = z;
      cartesian_move_single_step(current_pose, z_pose);

      // Update current pose after Z movement
      current_pose = move_group_robot_->getCurrentPose().pose;

      // Step 2: Move XY
      RCLCPP_INFO(LOGGER, "Step 2: Moving XY plane to [%.3f, %.3f]", x, y);
      geometry_msgs::msg::Pose xy_pose = current_pose;
      xy_pose.position.x = x;
      xy_pose.position.y = y;
      cartesian_move_single_step(current_pose, xy_pose);
    } else {
      // Single movement (only one axis or combination)
      RCLCPP_INFO(LOGGER, "Single movement to target");
      cartesian_move_single_step(current_pose, target_pose);
    }
  }

  void cartesian_move_single_step(const geometry_msgs::msg::Pose &from_pose,
                                  const geometry_msgs::msg::Pose &to_pose) {
    // Create waypoints for Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(from_pose);
    waypoints.push_back(to_pose);

    // Plan Cartesian path
    RCLCPP_INFO(LOGGER, "Planning Cartesian path...");
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_robot_->computeCartesianPath(waypoints, 0.01,
                                                              0.0, trajectory);

    RCLCPP_INFO(LOGGER, "Cartesian path planned: %.2f%% achieved",
                fraction * 100.0);

    if (fraction < 0.9) {
      RCLCPP_WARN(LOGGER, "Warning: Only %.2f%% of path could be planned!",
                  fraction * 100.0);
    }

    // Execute trajectory
    RCLCPP_INFO(LOGGER, "Executing Cartesian trajectory...");
    execute_trajectory(trajectory);
  }

private:
  void execute_trajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group_robot_->execute(plan);
    RCLCPP_INFO(LOGGER, "Cartesian Trajectory Success!");
  }

  // member variables
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  std::shared_ptr<MoveGroupInterface> move_group_robot_;

  using JointModelGroup = moveit::core::JointModelGroup;
  const JointModelGroup *joint_model_group_robot_;

  using RobotStatePtr = moveit::core::RobotStatePtr;
  RobotStatePtr current_state_robot_;
  std::vector<double> joint_group_positions_robot_;
}; // class CartesianMove

void print_usage(const char *program_name) {
  std::cout << "\nUsage: " << program_name << "\n";
  std::cout << "\nParameters:\n";
  std::cout << "  x: Target X position (meters, default: 0.25)\n";
  std::cout << "  y: Target Y position (meters, default: 0.25)\n";
  std::cout << "  z: Target Z position (meters, default: 0.25)\n";
  std::cout << "\nBehavior:\n";
  std::cout << "  - Specify all the axes you want to move\n";
  std::cout << "  - Moves in straight line from current pose to target\n";
  std::cout << "  - Keeps current orientation\n";
  std::cout << "\nExamples:\n";
  std::cout << "  ros2 run moveit2_scripts cartesian_move --ros-args -p x:=0.3 "
               "-p y:=0.0 -p z:=0.2\n";
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  // Check for help flag
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    }
  }

  // initialize program node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("cartesian_move");

  // Initialize CartesianMove - constructor will set parameter defaults
  // from current position for any missing parameters
  CartesianMove cartesian_move(base_node);

  // Get parameter values (now with current position as defaults)
  double x = base_node->get_parameter("x").as_double();
  double y = base_node->get_parameter("y").as_double();
  double z = base_node->get_parameter("z").as_double();

  RCLCPP_INFO(base_node->get_logger(),
              "Target position: x=%.3f, y=%.3f, z=%.3f", x, y, z);

  cartesian_move.cartesian_move_to_target(x, y, z);
  rclcpp::shutdown();

  return 0;
}
