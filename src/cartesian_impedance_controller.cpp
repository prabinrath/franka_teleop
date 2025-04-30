// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_teleop/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka_teleop/pseudo_inversion.h>

namespace franka_teleop {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_teleop::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // space mouse setup
  spacemouse_sub_ = node_handle.subscribe("/spacenav/joy", 10, &CartesianImpedanceController::joyCallback, this);

  node_handle.getParam("/spacemouse_gains/cartesian_impedance/x_axis", ctrl_params_.x_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/y_axis", ctrl_params_.y_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/z_axis", ctrl_params_.z_axis);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/roll", ctrl_params_.roll);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/pitch", ctrl_params_.pitch);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/yaw", ctrl_params_.yaw);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/lpf_const_t", ctrl_params_.lpf_const_t);
  node_handle.getParam("/spacemouse_gains/cartesian_impedance/lpf_const_o", ctrl_params_.lpf_const_o);

  ROS_INFO("Ctrl Params: x_axis=%.3f, y_axis=%.3f, z_axis=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f, lpf_const_t=%.3f, lpf_const_o=%.3f", 
         ctrl_params_.x_axis, 
         ctrl_params_.y_axis, 
         ctrl_params_.z_axis, 
         ctrl_params_.roll, 
         ctrl_params_.pitch, 
         ctrl_params_.yaw, 
         ctrl_params_.lpf_const_t,
         ctrl_params_.lpf_const_o);

  return true;
}

void CartesianImpedanceController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  joy_ctrl[0] = msg->axes[0] * ctrl_params_.x_axis;
  joy_ctrl[1] = msg->axes[1] * ctrl_params_.y_axis;
  joy_ctrl[2] = msg->axes[2] * ctrl_params_.z_axis;

  joy_ctrl[3] = msg->axes[3] * ctrl_params_.roll;
  joy_ctrl[4] = msg->axes[4] * ctrl_params_.pitch;
  joy_ctrl[5] = msg->axes[5] * ctrl_params_.yaw;

}

void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // space mouse update
  std::array<double, 6> vel = {};
  vel[0] = (lpf_vel[0] * ctrl_params_.lpf_const_t) + (joy_ctrl[0] * (1.0 - ctrl_params_.lpf_const_t));
  vel[1] = (lpf_vel[1] * ctrl_params_.lpf_const_t) + (joy_ctrl[1] * (1.0 - ctrl_params_.lpf_const_t));
  vel[2] = (lpf_vel[2] * ctrl_params_.lpf_const_t) + (joy_ctrl[2] * (1.0 - ctrl_params_.lpf_const_t));
  vel[3] = (lpf_vel[3] * ctrl_params_.lpf_const_o) + (joy_ctrl[3] * (1.0 - ctrl_params_.lpf_const_o));
  vel[4] = (lpf_vel[4] * ctrl_params_.lpf_const_o) + (joy_ctrl[4] * (1.0 - ctrl_params_.lpf_const_o));
  vel[5] = (lpf_vel[5] * ctrl_params_.lpf_const_o) + (joy_ctrl[5] * (1.0 - ctrl_params_.lpf_const_o));
  lpf_vel = vel;

  // TODO: the maths needs more analysis here
  if (std::abs(vel[0]) + std::abs(vel[1]) + std::abs(vel[2]) > 0.05) {
    Eigen::Vector3d translation_error(-vel[0], vel[1], vel[2]);
    position_d_ = position - transform.rotation() * translation_error;
  }
  if (std::abs(vel[3]) + std::abs(vel[4]) + std::abs(vel[5]) > 0.1) {
    Eigen::Quaterniond orientation_error =
      Eigen::AngleAxisd(-vel[3], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(vel[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(vel[5], Eigen::Vector3d::UnitZ());
    orientation_d_ = orientation.inverse() * orientation_error;
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::complianceParamCallback(
    franka_teleop::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

}  // namespace franka_teleop

PLUGINLIB_EXPORT_CLASS(franka_teleop::CartesianImpedanceController,
                       controller_interface::ControllerBase)