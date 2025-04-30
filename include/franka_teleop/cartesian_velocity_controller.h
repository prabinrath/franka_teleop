// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>


namespace franka_teleop {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void grasp();
  void drop();

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  ros::Subscriber spacemouse_sub_;
  std::unique_ptr< actionlib::SimpleActionClient<franka_gripper::GraspAction> > grasp_action_;
  std::unique_ptr< actionlib::SimpleActionClient<franka_gripper::MoveAction> > move_action_;

  struct ControlParams {
    double x_axis;
    double y_axis;
    double z_axis;
    double roll;
    double pitch;
    double yaw;
    double lpf_const;
  } ctrl_params_;
  std::array<double, 6> joy_ctrl{};
  std::array<double, 6> lpf_vel{};
  bool grasp_toggle;
  bool is_grasped;
};

}  // namespace franka_teleop
