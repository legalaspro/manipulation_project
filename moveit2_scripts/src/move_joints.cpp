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

class MoveJoints {
public:
  MoveJoints(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Move Joints...");

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
    RCLCPP_INFO(LOGGER, "Class Initialized: Move Joints");
  }

  ~MoveJoints() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Move Joints");
  }

  void move_to_joint_values(double j0, double j1, double j2, double j3,
                            double j4, double j5) {
    RCLCPP_INFO(LOGGER,
                "Moving to joint values: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                j0, j1, j2, j3, j4, j5);

    // setup the joint value target
    setup_joint_value_target(j0, j1, j2, j3, j4, j5);

    // plan trajectory
    RCLCPP_INFO(LOGGER, "Planning trajectory...");
    plan_trajectory_kinematics();

    // execute trajectory
    RCLCPP_INFO(LOGGER, "Executing trajectory...");
    execute_trajectory_kinematics();
  }

private:
  void setup_joint_value_target(double j0, double j1, double j2, double j3,
                                double j4, double j5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = j0; // Shoulder Pan
    joint_group_positions_robot_[1] = j1; // Shoulder Lift
    joint_group_positions_robot_[2] = j2; // Elbow
    joint_group_positions_robot_[3] = j3; // Wrist 1
    joint_group_positions_robot_[4] = j4; // Wrist 2
    joint_group_positions_robot_[5] = j5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
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
  bool plan_success_;
}; // class MoveJoints

void print_usage(const char *program_name) {
  std::cout << "\nUsage: " << program_name << " \"j0,j1,j2,j3,j4,j5\"\n";
  std::cout << "\nArguments:\n";
  std::cout
      << "  Array of 6 joint angles in radians (comma-separated, in quotes)\n";
  std::cout << "  j0: Shoulder Pan\n";
  std::cout << "  j1: Shoulder Lift\n";
  std::cout << "  j2: Elbow\n";
  std::cout << "  j3: Wrist 1\n";
  std::cout << "  j4: Wrist 2\n";
  std::cout << "  j5: Wrist 3\n";
  std::cout << "\nExamples:\n";
  std::cout << "  " << program_name
            << " \"0.0,-1.5708,0.0,-1.5708,0.0,0.0\"    # Home position\n";
  std::cout << "  " << program_name
            << " \"1.5708,-1.5708,0.0,-1.5708,0.0,0.0\"  # Rotated shoulder\n";
  std::cout << std::endl;
}

std::vector<double> parse_joint_array(const std::string &array_str) {
  std::vector<double> joints;

  // Parse comma-separated values
  std::stringstream ss(array_str);
  std::string token;

  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
    token.erase(token.find_last_not_of(" \t\n\r\f\v") + 1);

    if (!token.empty()) {
      try {
        joints.push_back(std::stod(token));
      } catch (const std::exception &e) {
        throw std::runtime_error("Invalid joint value: " + token);
      }
    }
  }

  if (joints.size() != 6) {
    throw std::runtime_error("Expected 6 joint values, got " +
                             std::to_string(joints.size()));
  }

  return joints;
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
    std::cerr << "Error: Insufficient arguments!\n";
    print_usage(argv[0]);
    return 1;
  }

  // Parse joint array
  std::vector<double> joints;
  try {
    joints = parse_joint_array(argv[1]);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  // initialize program node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("move_joints");
  MoveJoints move_joints(base_node);
  move_joints.move_to_joint_values(joints[0], joints[1], joints[2], joints[3],
                                   joints[4], joints[5]);
  rclcpp::shutdown();

  return 0;
}

// End of Code
