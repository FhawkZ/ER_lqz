// Copyright 2025 SERL Team
//
// ROS2 Real Robot Version of SERL Cartesian Impedance Controller
// Adapted for real Franka robots with franka_ros2
//
// Reference: https://github.com/rail-berkeley/serl_franka_controllers

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <franka_semantic_components/franka_robot_model.hpp>

namespace serl_franka_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CartesianImpedanceControllerReal : public controller_interface::ControllerInterface {
public:
  CartesianImpedanceControllerReal();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  // Robot arm ID
  std::string arm_id_;
  
  // Semantic component for accessing robot model
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  
  // Number of joints
  static constexpr int num_joints = 7;
  
  // Control parameters (loaded from YAML)
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> Ki_;
  Eigen::Matrix<double, 6, 6> Ki_target_;
  
  double nullspace_stiffness_{0.2};
  double nullspace_stiffness_target_{0.2};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  
  // Error clipping limits
  Eigen::Matrix<double, 3, 1> translational_clip_min_;
  Eigen::Matrix<double, 3, 1> translational_clip_max_;
  Eigen::Matrix<double, 3, 1> rotational_clip_min_;
  Eigen::Matrix<double, 3, 1> rotational_clip_max_;
  
  // Filter parameter
  double filter_params_{0.005};
  
  // Torque saturation
  const double delta_tau_max_{10.0};
  
  // Target pose
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  
  // Nullspace configuration
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  
  // Error tracking
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> error_i_;  // Integral error
  
  // Previous torque for rate limiting
  Eigen::Matrix<double, 7, 1> tau_J_d_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  
  // Publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> 
    publisher_jacobian_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> 
    publisher_error_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    publisher_current_pose_;
  
  // Callbacks
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // Helper functions
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d);
  
  void updateStiffnessDampingFiltered();
};

}  // namespace serl_franka_controllers

