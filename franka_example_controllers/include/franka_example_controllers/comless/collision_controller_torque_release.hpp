// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>
#include "multi_mode_control_msgs/srv/joint_collision_goal.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The collision controller implements joint impedance control and accepts joint trajectory goals via a service.
 */
class CollisionControllerTorqueRelease : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  void updateJointStates();

  // Ruckig
  ruckig::Ruckig<7> otg_{0.001};
  
  struct Waypoint {
      Vector7d q;
      Vector7d dq;
      Vector7d ddq;
  };
  std::vector<Waypoint> trajectory_buffer_;
  size_t current_waypoint_index_{0};
  bool trajectory_running_{false};

  Vector7d q_d_;
  Vector7d dq_d_;
  Vector7d ddq_d_;

  rclcpp::Service<multi_mode_control_msgs::srv::JointCollisionGoal>::SharedPtr goal_service_;
  void goalCallback(const std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Request> request,
                    std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Response> response);
};

}  // namespace franka_example_controllers