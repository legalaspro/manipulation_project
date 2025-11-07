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

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class MoveToPose {
public:
  MoveToPose(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Move To Pose...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
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
    RCLCPP_INFO(LOGGER, "Class Initialized: Move To Pose");
  }

  ~MoveToPose() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Move To Pose");
  }

  void move_to_target(double x, double y, double z, double qx = -1.0,
                      double qy = 0.0, double qz = 0.0, double qw = 0.0) {
    RCLCPP_INFO(LOGGER, "Moving to target position: [%.3f, %.3f, %.3f]", x, y,
                z);
    RCLCPP_INFO(LOGGER,
                "Target orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]", qx,
                qy, qz, qw);

    // setup the goal pose target
    setup_goal_pose_target(x, y, z, qx, qy, qz, qw);

    // plan trajectory
    RCLCPP_INFO(LOGGER, "Planning trajectory...");
    plan_trajectory_kinematics();

    // execute trajectory
    RCLCPP_INFO(LOGGER, "Executing trajectory...");
    execute_trajectory_kinematics();
  }

private:
  void setup_goal_pose_target(double x, double y, double z, double qx,
                              double qy, double qz, double qw) {
    target_pose_.orientation.x = qx;
    target_pose_.orientation.y = qy;
    target_pose_.orientation.z = qz;
    target_pose_.orientation.w = qw;
    target_pose_.position.x = x;
    target_pose_.position.y = y;
    target_pose_.position.z = z;
    move_group_robot_->setPoseTarget(target_pose_);
  }

  void plan_trajectory_kinematics() {
    plan_success_ = (move_group_robot_->plan(plan_robot_) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_) {
      move_group_robot_->execute(plan_robot_);
      RCLCPP_INFO(LOGGER, "Robot Trajectory Success!");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Trajectory Failed!");
    }
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

  MoveGroupInterface::Plan plan_robot_;
  geometry_msgs::msg::Pose target_pose_;
  bool plan_success_;
}; // class MoveToPose

void print_usage(const char *program_name) {
  std::cout << "\nUsage: " << program_name << " <x> <y> <z> [qx qy qz qw]\n";
  std::cout << "\nArguments:\n";
  std::cout << "  x, y, z    : Target position coordinates (meters)\n";
  std::cout << "  qx qy qz qw: Target orientation quaternion (optional)\n";
  std::cout << "               Default: qx=-1.0, qy=0.0, qz=0.0, qw=0.0 "
               "(grasping pose)\n";
  std::cout << "\nExamples:\n";
  std::cout << "  " << program_name
            << " 0.3 0.0 0.2                    # Uses default grasping "
               "orientation\n";
  std::cout << "  " << program_name
            << " 0.3 0.0 0.2 0.0 0.0 0.0 1.0    # Custom orientation\n";
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

  // Check minimum arguments (need at least x, y, z)
  if (argc < 4) {
    std::cerr << "Error: Insufficient arguments!\n";
    print_usage(argv[0]);
    return 1;
  }

  // Parse position arguments
  double x, y, z;
  try {
    x = std::stod(argv[1]);
    y = std::stod(argv[2]);
    z = std::stod(argv[3]);
  } catch (const std::exception &e) {
    std::cerr << "Error: Invalid position arguments! " << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  // Parse optional orientation arguments (default: grasping pose)
  double qx = -1.0, qy = 0.0, qz = 0.0, qw = 0.0;
  if (argc >= 8) {
    try {
      qx = std::stod(argv[4]);
      qy = std::stod(argv[5]);
      qz = std::stod(argv[6]);
      qw = std::stod(argv[7]);
    } catch (const std::exception &e) {
      std::cerr << "Error: Invalid orientation arguments! " << e.what() << "\n";
      print_usage(argv[0]);
      return 1;
    }
  } else if (argc > 4 && argc < 8) {
    std::cerr << "Error: Orientation requires all 4 quaternion values (qx qy "
                 "qz qw)!\n";
    print_usage(argv[0]);
    return 1;
  }

  // initialize program node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("move_to_pose");
  MoveToPose move_to_pose(base_node);
  move_to_pose.move_to_target(x, y, z, qx, qy, qz, qw);
  rclcpp::shutdown();

  return 0;
}
