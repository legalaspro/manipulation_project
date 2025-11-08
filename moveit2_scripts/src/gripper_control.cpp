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
static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control_node");
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

// Gripper parameters (Robotiq 85)
static const double GRIPPER_OPEN_RADIANS = 0.0; // Fully open
static const double GRIPPER_CLOSED_RADIANS =
    0.804; // Fully closed (0.804 based on the URDF)
static const double GRIPPER_MAX_WIDTH_M = 0.085; // Max opening in meters (85mm)

class GripperControl {
public:
  GripperControl(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Gripper Control...");

    // configure node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);
    move_group_gripper_->setGoalTolerance(0.0005);
    move_group_gripper_->setMaxVelocityScalingFactor(
        0.1); // Slow to 1% for precise/less jittery close
    move_group_gripper_->setMaxAccelerationScalingFactor(0.1);

    // get initial state of gripper
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_gripper_->getPlanningFrame().c_str());

    // get current state of gripper
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of gripper to current state
    move_group_gripper_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Class Initialized: Gripper Control");
  }

  ~GripperControl() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Gripper Control");
  }

  void control_gripper(double value, const std::string &mode) {
    if (mode == "radians") {
      RCLCPP_INFO(LOGGER, "Setting gripper to %.4f radians", value);
      set_gripper_radians(value);
    } else if (mode == "width") {
      RCLCPP_INFO(LOGGER, "Setting gripper to %.4f m width", value);
      double radians = width_m_to_radians(value);
      RCLCPP_INFO(LOGGER, "Converted to %.4f radians", radians);
      set_gripper_radians(radians);
    } else {
      RCLCPP_WARN(LOGGER, "Unknown mode: %s. Use 'radians' or 'width'",
                  mode.c_str());
    }
  }

private:
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const moveit::core::JointModelGroup *joint_model_group_gripper_;
  moveit::core::RobotStatePtr current_state_gripper_;
  std::vector<double> joint_group_positions_gripper_;
  MoveGroupInterface::Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  double width_m_to_radians(double width_m) {
    // Linear conversion: 0m = 0 radians (open), 0.085m = 0.8 radians (closed)
    double radians = ((GRIPPER_MAX_WIDTH_M - width_m) / GRIPPER_MAX_WIDTH_M) *
                     GRIPPER_CLOSED_RADIANS;
    // Clamp to valid range
    radians = std::max(GRIPPER_OPEN_RADIANS,
                       std::min(GRIPPER_CLOSED_RADIANS, radians));
    return radians;
  }

  void set_gripper_radians(double radians) {
    // Clamp to valid range
    radians = std::max(GRIPPER_OPEN_RADIANS,
                       std::min(GRIPPER_CLOSED_RADIANS, radians));

    joint_group_positions_gripper_[2] = radians;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);

    plan_trajectory_gripper();
    execute_trajectory_gripper();
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Command Success!");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Command Failed!");
    }
  }
};

void print_usage(const char *program_name) {
  std::cout << "\nUsage: " << program_name << " <value> [mode]\n";
  std::cout << "\nArguments:\n";
  std::cout << "  value: Gripper value (radians or meters depending on mode)\n";
  std::cout
      << "  mode: Control mode - 'radians' or 'width' (default: 'radians')\n";
  std::cout << "\nModes:\n";
  std::cout << "  radians: Direct radian value (0.0 = open, 0.8 = closed)\n";
  std::cout << "  width: Object width in meters (0.0 = open, 0.085 = closed)\n";
  std::cout << "\nExamples:\n";
  std::cout << "  " << program_name
            << " 0.0 radians           # Open gripper\n";
  std::cout << "  " << program_name
            << " 0.8 radians           # Close gripper\n";
  std::cout << "  " << program_name
            << " 0.4 radians           # Set to 0.4 radians\n";
  std::cout << "  " << program_name
            << " 0.03 width            # Gripper for 30mm object\n";
  std::cout << "  " << program_name
            << " 0.06 width            # Gripper for 60mm object\n";
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

  // Check minimum arguments
  if (argc < 2) {
    std::cerr << "Error: Need at least value argument!\n";
    print_usage(argv[0]);
    return 1;
  }

  // Parse arguments
  double value = 0.0;
  std::string mode = "radians";

  try {
    value = std::stod(argv[1]);
    mode = argv[2];
  } catch (const std::exception &e) {
    std::cerr << "Error: Invalid arguments! " << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  // initialize program node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("gripper_control");

  RCLCPP_INFO(base_node->get_logger(), "Gripper Control: value=%.4f, mode=%s",
              value, mode.c_str());

  GripperControl gripper_control(base_node);
  gripper_control.control_gripper(value, mode);
  rclcpp::shutdown();

  return 0;
}
