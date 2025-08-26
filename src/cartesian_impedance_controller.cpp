/*
Reference: 
  https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/cartesian_impedance_example_controller.cpp
*/

#include <franka_teleop/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_teleop/pseudo_inversion.h>
#include <ros/console.h>

namespace franka_teleop {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;
  publisher_franka_jacobian_.init(node_handle, "franka_jacobian", 1);

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

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
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_teleop::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // space mouse setup
  spacemouse_sub_ = node_handle.subscribe("/spacenav/joy", 10, &CartesianImpedanceController::joyCallback, this);
  grasp_action_ = std::make_unique< actionlib::SimpleActionClient<franka_gripper::GraspAction> >("/franka_gripper/grasp", true);
  move_action_ = std::make_unique< actionlib::SimpleActionClient<franka_gripper::MoveAction> >("/franka_gripper/move", true);

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
  
  ros::Duration(2.0).sleep();
  grasp_toggle = false;
  drop();
  is_grasped = false;

  return true;
}

void CartesianImpedanceController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
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
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  publishZeroJacobian(time);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // Clip translational error
  error_.head(3) << position - position_d_;
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  // Clip rotation error
  error_.tail(3) << -transform.linear() * error_.tail(3);
    for (int i = 0; i < 3; i++) {
    error_(i+3) = std::min(std::max(error_(i+3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  error_i.head(3) << (error_i.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i.tail(3) << (error_i.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq) - Ki_ * error_i);

  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;

  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * qe -
                        (2.0 * sqrt(nullspace_stiffness_)) * dqe);
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

  // apply deadband
  // for (double &a : vel) {
  //     if (std::abs(a) < deadband) {
  //         a = 0.0;
  //     }
  // }
  // bool all_zero = std::all_of(vel.begin(), vel.end(),
  //   [](double v){ return std::abs(v) < 1e-9; });
  // if (!all_zero) {
  //   position_d_target_[0] = position[0] + vel[0];
  //   position_d_target_[1] = position[1] + vel[1];
  //   position_d_target_[2] = position[2] + vel[2];
  //   error_i.setZero();
  //   Eigen::AngleAxisd Rx(vel[3],  Eigen::Vector3d::UnitX());
  //   Eigen::AngleAxisd Ry(vel[4], Eigen::Vector3d::UnitY());
  //   Eigen::AngleAxisd Rz(vel[5],   Eigen::Vector3d::UnitZ());
  //   Eigen::Quaterniond q_delta = Rz * Ry * Rx;   // Z * Y * X = intrinsic "xyz"
  //   // update orientation (global frame update, like scipy 'xyz')
  //   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  //   orientation_d_target_ = q_delta * orientation;
  //   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
  //     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  //   }
  //   orientation_d_target_.normalize();
  // }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      filter_params_ * joint1_nullspace_stiffness_target_ + (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;
}

void CartesianImpedanceController::publishZeroJacobian(const ros::Time& time) {
  if (publisher_franka_jacobian_.trylock()) {
      for (size_t i = 0; i < jacobian_array.size(); i++) {
        publisher_franka_jacobian_.msg_.zero_jacobian[i] = jacobian_array[i];
      }
      publisher_franka_jacobian_.unlockAndPublish();
    }
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
      << config.translational_damping * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << config.rotational_damping * Eigen::Matrix3d::Identity();
 
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  joint1_nullspace_stiffness_target_ = config.joint1_nullspace_stiffness;

  translational_clip_min_ << -config.translational_clip_neg_x, -config.translational_clip_neg_y, -config.translational_clip_neg_z;
  translational_clip_max_ << config.translational_clip_x, config.translational_clip_y, config.translational_clip_z;
  rotational_clip_min_ << -config.rotational_clip_neg_x, -config.rotational_clip_neg_y, -config.rotational_clip_neg_z;
  rotational_clip_max_ << config.rotational_clip_x, config.rotational_clip_y, config.rotational_clip_z;

  Ki_target_.setIdentity();
  Ki_target_.topLeftCorner(3, 3)
      << config.translational_Ki * Eigen::Matrix3d::Identity();
  Ki_target_.bottomRightCorner(3, 3)
      << config.rotational_Ki * Eigen::Matrix3d::Identity();
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  error_i.setZero();
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianImpedanceController::grasp() {
    franka_gripper::GraspGoal goal;
    goal.width = 0.0;
    goal.epsilon.inner = 0.08;
    goal.epsilon.outer = 0.08;
    goal.speed = 0.1;
    goal.force = 5.0;

    ROS_INFO("Grasping...");
    grasp_action_->sendGoal(goal);
}

void CartesianImpedanceController::drop() {
    franka_gripper::MoveGoal goal;
    goal.width = 0.08;
    goal.speed = 0.1;

    ROS_INFO("Dropping...");
    move_action_->sendGoal(goal);
}

}  // namespace franka_teleop

PLUGINLIB_EXPORT_CLASS(franka_teleop::CartesianImpedanceController,
                       controller_interface::ControllerBase)
