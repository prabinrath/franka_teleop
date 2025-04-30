// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_teleop/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_teleop {

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
    return false;
  }

  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");

  //   std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "CartesianVelocityController: Robot is not in the expected starting position "
  //           "for running this example. Run `roslaunch franka_teleop "
  //           "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
  //           "first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "CartesianVelocityController: Exception getting state handle: " << e.what());
  //   return false;
  // }

  spacemouse_sub_ = node_handle.subscribe("/spacenav/joy", 10, &CartesianVelocityController::joyCallback, this);
  grasp_action_ = std::make_unique< actionlib::SimpleActionClient<franka_gripper::GraspAction> >("/franka_gripper/grasp", true);
  move_action_ = std::make_unique< actionlib::SimpleActionClient<franka_gripper::MoveAction> >("/franka_gripper/move", true);
  
  grasp_action_->waitForServer();
  ROS_INFO("Connected to /franka_gripper/grasp action server.");
  move_action_->waitForServer();
  ROS_INFO("Connected to /franka_gripper/move action server.");
  
  // loading controller rosparams
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/x_axis", ctrl_params_.x_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/y_axis", ctrl_params_.y_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/z_axis", ctrl_params_.z_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/roll", ctrl_params_.roll);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/pitch", ctrl_params_.pitch);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/yaw", ctrl_params_.yaw);
  node_handle.getParam("/spacemouse_gains/cartesian_velocity/lpf_const", ctrl_params_.lpf_const);
    

  ROS_INFO("Ctrl Params: x_axis=%.3f, y_axis=%.3f, z_axis=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f, lpf_const=%.3f", 
         ctrl_params_.x_axis, 
         ctrl_params_.y_axis, 
         ctrl_params_.z_axis, 
         ctrl_params_.roll, 
         ctrl_params_.pitch, 
         ctrl_params_.yaw, 
         ctrl_params_.lpf_const);
  
  ros::Duration(2.0).sleep();
  grasp_toggle = false;
  drop();
  is_grasped = false;
  
  return true;
}

void CartesianVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  std::array<double, 6> vel = {};
  for (int j = 0; j < vel.size(); j++) {
    vel[j] = (lpf_vel[j] * ctrl_params_.lpf_const) + (joy_ctrl[j] * (1.0 - ctrl_params_.lpf_const));
  }

  lpf_vel = vel;
  std::array<double, 6> command = {{vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]}};
  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  joy_ctrl[0] = msg->axes[0] * ctrl_params_.x_axis;
  joy_ctrl[1] = msg->axes[1] * ctrl_params_.y_axis;
  joy_ctrl[2] = msg->axes[2] * ctrl_params_.z_axis;

  joy_ctrl[3] = msg->axes[3] * ctrl_params_.roll;
  joy_ctrl[4] = msg->axes[4] * ctrl_params_.pitch;
  joy_ctrl[5] = msg->axes[5] * ctrl_params_.yaw;

  if (msg->buttons[4] > 0 && !grasp_toggle) { // SHIFT key on the spacemouse
    if (is_grasped){
      drop();
      is_grasped = false;
    }
    else {
      grasp();
      is_grasped = true;
    }
    grasp_toggle = true;
  }
  if (msg->buttons[4] == 0) {
    grasp_toggle = false;
  }

}

void CartesianVelocityController::grasp() {
    franka_gripper::GraspGoal goal;
    goal.width = 0.0;
    goal.epsilon.inner = 0.08;
    goal.epsilon.outer = 0.08;
    goal.speed = 0.1;
    goal.force = 5.0;

    ROS_INFO("Grasping...");
    grasp_action_->sendGoal(goal);
}

void CartesianVelocityController::drop() {
    franka_gripper::MoveGoal goal;
    goal.width = 0.08;
    goal.speed = 0.1;

    ROS_INFO("Dropping...");
    move_action_->sendGoal(goal);
}

}  // namespace franka_teleop

PLUGINLIB_EXPORT_CLASS(franka_teleop::CartesianVelocityController,
                       controller_interface::ControllerBase)
