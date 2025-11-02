#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <detection_interfaces/msg/detected_objects.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

// offsets / “magic numbers”:
static constexpr double PREGRASP_Z_OFFSET = 0.20; // 20 cm above detected object
static constexpr double APPROACH_Z_DELTA = -0.085; // straight down 8 cm
static constexpr double RETREAT_Z_DELTA = +0.085;  // straight up 8 cm
static constexpr double GRIPPER_UPPER_LIMIT =
    0.804; // Upper limit set in the joint configuration
static constexpr double GRIPPER_MAX_OPEN_WIDTH =
    0.085; // Robotiq 85 gripper specs

class PickAndPlacePerception {
public:
  PickAndPlacePerception(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Perception...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    executor_thread_ = std::thread([this]() { this->executor_.spin(); });

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    move_group_gripper_->setGoalTolerance(0.0001);
    move_group_gripper_->setMaxVelocityScalingFactor(
        0.01); // Slow to 2.5% for precise/less jittery close
    move_group_gripper_->setMaxAccelerationScalingFactor(0.01);

    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // subscribe to object detection on move_group_node_
    object_promise_ = std::make_shared<std::promise<DetectedObject>>();
    object_future_ = object_promise_->get_future();
    object_sub_ = move_group_node_->create_subscription<DetectedObject>(
        "/object_detected", 10,
        std::bind(&PickAndPlacePerception::object_callback, this,
                  std::placeholders::_1));

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());

    for (const auto &name : move_group_robot_->getJointModelGroupNames())
      RCLCPP_INFO(LOGGER, "Available group: %s", name.c_str());

    // get current state of robot and gripper
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Perception");
  }

  ~PickAndPlacePerception() {
    executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Perception");
  }

  void execute_trajectory_plan() {
    using namespace std::chrono_literals;
    RCLCPP_INFO(LOGGER, "Waiting for first /object_detected...");

    while (rclcpp::ok()) {
      if (object_future_.wait_for(5s) == std::future_status::ready) {
        break; // Object is ready, proceed
      } else {
        RCLCPP_INFO(LOGGER, "No object detected yet, waiting another 5s...");
      }
    }

    DetectedObject obj = object_future_.get(); // safe copy

    const double pre_x = obj.position.x + obj.height * 0.5;
    const double pre_y = obj.position.y - obj.width * 0.5;
    const double pre_z =
        obj.position.z + obj.thickness * 0.5 + PREGRASP_Z_OFFSET;
    const double target_close_rad = GRIPPER_UPPER_LIMIT *
                                    (GRIPPER_MAX_OPEN_WIDTH - obj.height) /
                                    GRIPPER_MAX_OPEN_WIDTH;
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place Perception...");

    // 1. go to pregrasp
    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position (%.3f, %.3f, %.3f)...",
                pre_x, pre_y, pre_z);
    // return;
    setup_goal_pose_target(pre_x, pre_y, pre_z, -1.000, +0.000, +0.000, +0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    // 2. open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // 3. approach straight down
    RCLCPP_INFO(LOGGER, "Approaching object...");
    setup_waypoints_target(+0.000, +0.000, APPROACH_Z_DELTA);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // 4. close the gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper... (target %f)", target_close_rad);
    setup_joint_value_gripper(target_close_rad);
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 5. retreat
    RCLCPP_INFO(LOGGER, "Retreating...");
    setup_waypoints_target(+0.000, +0.000, RETREAT_Z_DELTA);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // 6. rotate shoulder
    RCLCPP_INFO(LOGGER, "Rotate Shoulder Joint 180 degrees...");
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    setup_joint_value_target(
        joint_group_positions_robot_[0] + M_PI, // add 180 degrees
        joint_group_positions_robot_[1], joint_group_positions_robot_[2],
        joint_group_positions_robot_[3], joint_group_positions_robot_[4],
        joint_group_positions_robot_[5]);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    // 7. open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 8. Going to Initial Position
    RCLCPP_INFO(LOGGER, "Going to Initial Position...");
    setup_joint_value_target(+0.0000, -1.5708, +0.0000, -1.5708, +0.0000,
                             +0.0000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    RCLCPP_INFO(LOGGER, "Pick And Place Perception Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;
  using DetectedObject = detection_interfaces::msg::DetectedObjects;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_{false};

  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_{false};

  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_{0.0};
  const double end_effector_step_{0.01};
  double plan_fraction_robot_{0.0};

  // detection
  std::shared_ptr<std::promise<DetectedObject>> object_promise_;
  std::future<DetectedObject> object_future_;
  rclcpp::Subscription<DetectedObject>::SharedPtr object_sub_;
  bool object_received_{false};

  void object_callback(const DetectedObject::SharedPtr msg) {
    if (!object_received_) {
      object_received_ = true;
      object_promise_->set_value(*msg);
      object_sub_.reset();
      RCLCPP_INFO(LOGGER, "Received object id %d at (%.3f, %.3f, %.3f)",
                  msg->object_id, msg->position.x, msg->position.y,
                  msg->position.z);
      RCLCPP_INFO(LOGGER, "Height: %.3f Width: %.3f Thickness: %.3f",
                  msg->height, msg->width, msg->thickness);
    }
  }

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    joint_group_positions_robot_.resize(6);
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    cartesian_waypoints_.clear();
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_robot_);
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
  }

  void setup_joint_value_gripper(float angle) {
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

}; // class PickAndPlacePerception

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place_perception");
  PickAndPlacePerception pick_and_place(base_node);
  pick_and_place.execute_trajectory_plan();
  rclcpp::shutdown();
  return 0;
}