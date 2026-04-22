// Copyright 2025 SERL Team
//
// ROS2 Real Robot Version of SERL Cartesian Impedance Controller
// Adapted for real Franka robots with franka_ros2
//
// Reference: https://github.com/rail-berkeley/serl_franka_controllers

#include <serl_franka_controllers/cartesian_impedance_controller_real.hpp>
#include <serl_franka_controllers/pseudo_inversion.h>

#include <cmath>
#include <memory>
#include <algorithm>

namespace serl_franka_controllers {

CartesianImpedanceControllerReal::CartesianImpedanceControllerReal()
  : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CartesianImpedanceControllerReal::on_init() {
  try {
    // Declare parameters with default values
    auto_declare<std::string>("arm_id", "panda");
    
    // Cartesian stiffness (diagonal: x, y, z, rx, ry, rz)
    // Note: Using same format as ROS1 version for compatibility
    auto_declare<double>("translational_stiffness", 2000.0);
    auto_declare<double>("rotational_stiffness", 150.0);
    
    // Cartesian damping (usually 2*sqrt(stiffness) for critical damping)
    auto_declare<double>("translational_damping", 89.0);
    auto_declare<double>("rotational_damping", 7.0);
    
    // Integral gains
    auto_declare<double>("translational_Ki", 0.0);
    auto_declare<double>("rotational_Ki", 0.0);
    
    // Nullspace stiffness
    auto_declare<double>("nullspace_stiffness", 0.2);
    auto_declare<double>("joint1_nullspace_stiffness", 20.0);
    
    // Clipping limits
    auto_declare<double>("translational_clip_neg_x", 0.01);
    auto_declare<double>("translational_clip_neg_y", 0.01);
    auto_declare<double>("translational_clip_neg_z", 0.01);
    auto_declare<double>("translational_clip_x", 0.01);
    auto_declare<double>("translational_clip_y", 0.01);
    auto_declare<double>("translational_clip_z", 0.01);
    
    auto_declare<double>("rotational_clip_neg_x", 0.05);
    auto_declare<double>("rotational_clip_neg_y", 0.05);
    auto_declare<double>("rotational_clip_neg_z", 0.05);
    auto_declare<double>("rotational_clip_x", 0.05);
    auto_declare<double>("rotational_clip_y", 0.05);
    auto_declare<double>("rotational_clip_z", 0.05);
    
    // Filter parameter
    auto_declare<double>("filter_params", 0.005);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerReal::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Get arm_id parameter
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  
  // Initialize semantic component for accessing robot model
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    arm_id_ + "/" + k_robot_model_interface_name, 
    arm_id_ + "/" + k_robot_state_interface_name);
  
  // Load stiffness and damping (scalar values as in ROS1)
  double trans_stiff = get_node()->get_parameter("translational_stiffness").as_double();
  double rot_stiff = get_node()->get_parameter("rotational_stiffness").as_double();
  double trans_damp = get_node()->get_parameter("translational_damping").as_double();
  double rot_damp = get_node()->get_parameter("rotational_damping").as_double();
  double trans_ki = get_node()->get_parameter("translational_Ki").as_double();
  double rot_ki = get_node()->get_parameter("rotational_Ki").as_double();
  
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  Ki_.setZero();
  
  // Set diagonal elements
  cartesian_stiffness_.topLeftCorner(3, 3) = trans_stiff * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) = rot_stiff * Eigen::Matrix3d::Identity();
  
  cartesian_damping_.topLeftCorner(3, 3) = trans_damp * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) = rot_damp * Eigen::Matrix3d::Identity();
  
  Ki_.topLeftCorner(3, 3) = trans_ki * Eigen::Matrix3d::Identity();
  Ki_.bottomRightCorner(3, 3) = rot_ki * Eigen::Matrix3d::Identity();
  
  cartesian_stiffness_target_ = cartesian_stiffness_;
  cartesian_damping_target_ = cartesian_damping_;
  Ki_target_ = Ki_;
  
  // Load nullspace stiffness
  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  joint1_nullspace_stiffness_ = get_node()->get_parameter("joint1_nullspace_stiffness").as_double();
  nullspace_stiffness_target_ = nullspace_stiffness_;
  joint1_nullspace_stiffness_target_ = joint1_nullspace_stiffness_;
  
  // Load clipping limits
  translational_clip_min_(0) = -get_node()->get_parameter("translational_clip_neg_x").as_double();
  translational_clip_min_(1) = -get_node()->get_parameter("translational_clip_neg_y").as_double();
  translational_clip_min_(2) = -get_node()->get_parameter("translational_clip_neg_z").as_double();
  
  translational_clip_max_(0) = get_node()->get_parameter("translational_clip_x").as_double();
  translational_clip_max_(1) = get_node()->get_parameter("translational_clip_y").as_double();
  translational_clip_max_(2) = get_node()->get_parameter("translational_clip_z").as_double();
  
  rotational_clip_min_(0) = -get_node()->get_parameter("rotational_clip_neg_x").as_double();
  rotational_clip_min_(1) = -get_node()->get_parameter("rotational_clip_neg_y").as_double();
  rotational_clip_min_(2) = -get_node()->get_parameter("rotational_clip_neg_z").as_double();
  
  rotational_clip_max_(0) = get_node()->get_parameter("rotational_clip_x").as_double();
  rotational_clip_max_(1) = get_node()->get_parameter("rotational_clip_y").as_double();
  rotational_clip_max_(2) = get_node()->get_parameter("rotational_clip_z").as_double();
  
  filter_params_ = get_node()->get_parameter("filter_params").as_double();
  
  // Initialize target pose and errors
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  
  error_.setZero();
  error_i_.setZero();
  q_d_nullspace_.setZero();
  tau_J_d_.setZero();
  
  // Create subscribers
  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/equilibrium_pose", 10,
    std::bind(&CartesianImpedanceControllerReal::equilibriumPoseCallback, this, std::placeholders::_1));
  
  // Create publishers
  publisher_jacobian_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian", 10));
  
  publisher_error_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/error", 10));
  
  publisher_current_pose_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10));
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian Impedance Controller (Real Robot) configured successfully");
  RCLCPP_INFO(get_node()->get_logger(), "  Arm ID: %s", arm_id_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "  Translational stiffness: %.1f", trans_stiff);
  RCLCPP_INFO(get_node()->get_logger(), "  Rotational stiffness: %.1f", rot_stiff);
  RCLCPP_INFO(get_node()->get_logger(), "  Nullspace stiffness: %.3f", nullspace_stiffness_);
  RCLCPP_INFO(get_node()->get_logger(), "  Clipping limits: trans=[%.3f, %.3f] m, rot=[%.3f, %.3f] rad",
              translational_clip_min_(0), translational_clip_max_(0),
              rotational_clip_min_(0), rotational_clip_max_(0));
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
CartesianImpedanceControllerReal::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // We need effort command interfaces for all 7 joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  
  return config;
}

controller_interface::InterfaceConfiguration 
CartesianImpedanceControllerReal::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // We need joint position and velocity interfaces
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  
  // We also need the robot model interfaces
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  
  return config;
}

controller_interface::CallbackReturn CartesianImpedanceControllerReal::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Assign loaned state interfaces to semantic component
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  
  // Get initial robot state
  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  
  // Set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  
  // Set nullspace equilibrium configuration to current q
  // Read from state_interfaces by finding position interfaces
  for (size_t i = 0; i < num_joints; ++i) {
    const std::string joint_name = arm_id_ + "_joint" + std::to_string(i + 1);
    auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                           [&joint_name](const auto& interface) {
                             return interface.get_prefix_name() == joint_name &&
                                    interface.get_interface_name() == "position";
                           });
    if (it != state_interfaces_.end()) {
      q_d_nullspace_(i) = it->get_value();
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian Impedance Controller (Real Robot) activated");
  RCLCPP_INFO(get_node()->get_logger(), "  Initial EE position: [%.3f, %.3f, %.3f]",
              position_d_(0), position_d_(1), position_d_(2));
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerReal::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Release state interfaces
  franka_robot_model_->release_interfaces();
  
  // Reset all command interfaces to zero torque
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian Impedance Controller (Real Robot) deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceControllerReal::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/) {
  
  // ============================================================================
  // Read current state from robot
  // ============================================================================
  
  // Get joint positions and velocities from state_interfaces
  // Find interfaces by name to ensure correct ordering
  Eigen::Matrix<double, 7, 1> q, dq;
  for (size_t i = 0; i < num_joints; ++i) {
    const std::string joint_name = arm_id_ + "_joint" + std::to_string(i + 1);
    
    auto pos_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                [&joint_name](const auto& interface) {
                                  return interface.get_prefix_name() == joint_name &&
                                         interface.get_interface_name() == "position";
                                });
    auto vel_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                [&joint_name](const auto& interface) {
                                  return interface.get_prefix_name() == joint_name &&
                                         interface.get_interface_name() == "velocity";
                                });
    
    if (pos_it != state_interfaces_.end() && vel_it != state_interfaces_.end()) {
      q(i) = pos_it->get_value();
      dq(i) = vel_it->get_value();
    }
  }
  
  // Get end effector pose
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_array.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  
  // Get Jacobian (6x7 in column-major)
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  // Get Coriolis forces
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  
  // ============================================================================
  // Update stiffness and damping with filtering
  // ============================================================================
  updateStiffnessDampingFiltered();
  
  // ============================================================================
  // Compute pose error with SERL's reference limiting
  // ============================================================================
  
  // Position error
  error_.head(3) << position - position_d_;
  
  // Clip translational error
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), 
                         translational_clip_max_(i));
  }
  
  // Orientation error (quaternion difference)
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  
  // Transform to base frame and clip
  error_.tail(3) << -transform.linear() * error_.tail(3);
  
  // Clip rotational error
  for (int i = 0; i < 3; i++) {
    error_(i+3) = std::min(std::max(error_(i+3), rotational_clip_min_(i)), 
                           rotational_clip_max_(i));
  }
  
  // Update integral error with anti-windup
  error_i_.head(3) << (error_i_.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i_.tail(3) << (error_i_.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);
  
  // ============================================================================
  // Compute control law
  // ============================================================================
  
  // Task-space control torque
  Eigen::VectorXd tau_task(7);
  tau_task << jacobian.transpose() *
              (-cartesian_stiffness_ * error_ - 
               cartesian_damping_ * (jacobian * dq) - 
               Ki_ * error_i_);
  
  // Null-space control
  // Compute pseudo-inverse of J^T for null-space projection
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  
  // Null-space error
  Eigen::Matrix<double, 7, 1> qe = q_d_nullspace_ - q;
  Eigen::Matrix<double, 7, 1> dqe = dq;
  
  // Apply special stiffness to joint 1 (for better control of elbow configuration)
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe.head(1) << dqe.head(1) * 2.0 * std::sqrt(joint1_nullspace_stiffness_);
  
  // Null-space torque
  Eigen::VectorXd tau_nullspace(7);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - 
                    jacobian.transpose() * jacobian_transpose_pinv) *
                   (nullspace_stiffness_ * qe - 
                    (2.0 * std::sqrt(nullspace_stiffness_)) * dqe);
  
  // Total desired torque
  Eigen::VectorXd tau_d(7);
  tau_d << tau_task + tau_nullspace + coriolis;
  
  // Saturate torque rate to prevent abrupt changes
  Eigen::Matrix<double, 7, 1> tau_d_saturated = saturateTorqueRate(tau_d, tau_J_d_);
  
  // ============================================================================
  // Send commands
  // ============================================================================
  for (size_t i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_saturated(i));
  }
  
  // Update previous command
  tau_J_d_ = tau_d_saturated;
  
  // ============================================================================
  // Publish debug info
  // ============================================================================
  if (publisher_jacobian_->trylock()) {
    publisher_jacobian_->msg_.data.resize(42);
    // Jacobian is stored in column-major format, directly copy from array
    for (size_t i = 0; i < 42; ++i) {
      publisher_jacobian_->msg_.data[i] = jacobian_array[i];
    }
    publisher_jacobian_->unlockAndPublish();
  }
  
  if (publisher_error_->trylock()) {
    publisher_error_->msg_.data.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      publisher_error_->msg_.data[i] = error_(i);
    }
    publisher_error_->unlockAndPublish();
  }
  
  // Publish current end-effector pose
  if (publisher_current_pose_->trylock()) {
    publisher_current_pose_->msg_.header.stamp = get_node()->now();
    publisher_current_pose_->msg_.header.frame_id = arm_id_ + "_link0";
    
    publisher_current_pose_->msg_.pose.position.x = position(0);
    publisher_current_pose_->msg_.pose.position.y = position(1);
    publisher_current_pose_->msg_.pose.position.z = position(2);
    
    publisher_current_pose_->msg_.pose.orientation.w = orientation.w();
    publisher_current_pose_->msg_.pose.orientation.x = orientation.x();
    publisher_current_pose_->msg_.pose.orientation.y = orientation.y();
    publisher_current_pose_->msg_.pose.orientation.z = orientation.z();
    
    publisher_current_pose_->unlockAndPublish();
  }
  
  return controller_interface::return_type::OK; 
}

void CartesianImpedanceControllerReal::equilibriumPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  
  // Update target pose
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, 
                                     msg->pose.orientation.y,
                                     msg->pose.orientation.z, 
                                     msg->pose.orientation.w;
  
  // Handle quaternion sign ambiguity
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  
  // Reset integral error on new target
  error_i_.setZero();
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceControllerReal::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  
  for (size_t i = 0; i < 7; ++i) {
    double difference = tau_d_calculated(i) - tau_J_d(i);
    tau_d_saturated(i) = tau_J_d(i) + std::max(std::min(difference, delta_tau_max_), 
                                                -delta_tau_max_);
  }
  
  return tau_d_saturated;
}

void CartesianImpedanceControllerReal::updateStiffnessDampingFiltered() {
  // Low-pass filter for smooth parameter changes
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + 
                         (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + 
                       (1.0 - filter_params_) * cartesian_damping_;
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;
  
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + 
                         (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ = filter_params_ * joint1_nullspace_stiffness_target_ + 
                                (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  
  // Update target pose with filtering
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

}  // namespace serl_franka_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  serl_franka_controllers::CartesianImpedanceControllerReal,
  controller_interface::ControllerInterface)

