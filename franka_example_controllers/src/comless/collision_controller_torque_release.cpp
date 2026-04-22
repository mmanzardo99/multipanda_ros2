#include <franka_example_controllers/comless/collision_controller_torque_release.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CollisionControllerTorqueRelease::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CollisionControllerTorqueRelease::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type CollisionControllerTorqueRelease::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  
  Vector7d tau_d_calculated;

  if (trajectory_running_) {
    if (current_waypoint_index_ < trajectory_buffer_.size()) {
      auto& waypoint = trajectory_buffer_[current_waypoint_index_];
      q_d_ = waypoint.q;
      dq_d_ = waypoint.dq;
      ddq_d_ = waypoint.ddq;
      current_waypoint_index_++;
      
      // Joint Impedance with Feedforward Acceleration: tau = M * ddq_d + K*(q_d - q) + D*(dq_d - dq) + C
      auto mass_matrix_array = franka_robot_model_->getMassMatrix();
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(mass_matrix_array.data());

      tau_d_calculated = M * ddq_d_ + k_gains_.cwiseProduct(q_d_ - q_) + d_gains_.cwiseProduct(dq_d_ - dq_filtered_) + coriolis;
    } else {
      trajectory_running_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "Final position reached. Trajectory execution finished.");
      tau_d_calculated = coriolis;
    }
  } else {
    tau_d_calculated = coriolis;
  }

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CollisionControllerTorqueRelease::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});

    goal_service_ = get_node()->create_service<multi_mode_control_msgs::srv::JointCollisionGoal>(
        "~/joint_collision_goal",
        std::bind(&CollisionControllerTorqueRelease::goalCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CollisionControllerTorqueRelease::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CollisionControllerTorqueRelease::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_d_ = q_;
  dq_d_.setZero();
  ddq_d_.setZero();
  trajectory_running_ = false;
  return CallbackReturn::SUCCESS;
}

void CollisionControllerTorqueRelease::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void CollisionControllerTorqueRelease::goalCallback(
    const std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Request> request,
    std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Response> response) {
  
  if (trajectory_running_) {
    response->success = false;
    response->message = "A trajectory is already in execution.";
    return;
  }

  ruckig::InputParameter<7> ruckig_input;
  for (int i = 0; i < num_joints; ++i) {
    ruckig_input.current_position[i] = q_(i);
    ruckig_input.current_velocity[i] = 0.0;
    ruckig_input.current_acceleration[i] = 0.0;

    ruckig_input.target_position[i] = request->q[i];
    ruckig_input.target_velocity[i] = request->dq[i];
    ruckig_input.target_acceleration[i] = request->qdd[i];

    ruckig_input.max_velocity[i] = (request->max_velocity[i] > 0.0) ? request->max_velocity[i] : 2.0;
    ruckig_input.max_acceleration[i] = (request->max_acceleration[i] > 0.0) ? request->max_acceleration[i] : 1.0;
    ruckig_input.max_jerk[i] = (request->max_jerk[i] > 0.0) ? request->max_jerk[i] : 10.0;
  }

  if (request->duration > 0.0) {
      ruckig_input.minimum_duration = request->duration;
      ruckig_input.duration_discretization = ruckig::DurationDiscretization::Continuous;
  }

  ruckig::Trajectory<7> trajectory;
  auto result = otg_.calculate(ruckig_input, trajectory);
  
  if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
      trajectory_buffer_.clear();
      double duration = trajectory.get_duration();
      // Sample at 1kHz
      std::array<double, 7> q_sample;
      std::array<double, 7> dq_sample;
      std::array<double, 7> ddq_sample;
      for (double t = 0.001; t <= duration; t += 0.001) {
          Waypoint wp;
          trajectory.at_time(t, q_sample, dq_sample, ddq_sample);
          for(int i=0; i<7; ++i) { 
              wp.q(i) = q_sample[i]; 
              wp.dq(i) = dq_sample[i]; 
              wp.ddq(i) = ddq_sample[i];
          }
          trajectory_buffer_.push_back(wp);
      }
      // Ensure the final point is included
      Waypoint final_wp;
      trajectory.at_time(duration, q_sample, dq_sample, ddq_sample);
      for(int i=0; i<7; ++i) { 
          final_wp.q(i) = q_sample[i]; 
          final_wp.dq(i) = dq_sample[i]; 
          final_wp.ddq(i) = ddq_sample[i];
      }
      trajectory_buffer_.push_back(final_wp);

      RCLCPP_INFO(get_node()->get_logger(), "Trajectory generated. Requested duration: %.3f, Ruckig duration: %.3f, Waypoints: %ld", 
                  request->duration, duration, trajectory_buffer_.size());

      current_waypoint_index_ = 0;
      trajectory_running_ = true;
      response->success = true;
      response->message = "Trajectory generated and execution started.";
  } else {
      response->success = false;
      response->message = "Failed to generate trajectory.";
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CollisionControllerTorqueRelease,
                       controller_interface::ControllerInterface)
