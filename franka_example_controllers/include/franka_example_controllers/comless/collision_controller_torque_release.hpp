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

// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The collision controller implements joint impedance control and accepts joint trajectory goals via a service.
 */
class CollisionControllerTorqueRelease : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
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

  bool post_impact_measurement_{false};
  double post_impact_time_{0.0};
  double t1_measure_{0.005};
  double t2_measure_{0.015};
  Vector7d post_impact_q_t1_;
  bool post_impact_t1_recorded_{false};
  Eigen::Matrix<double, 6, 7> J_target_post_impact_;

  // Pinocchio
  pinocchio::Model model_pin_;
  pinocchio::Data data_pin_;
  std::string robot_description_path_;

  Eigen::Vector3d u_d_;
  std::string target_link_name_;
  Eigen::Vector3d target_point_;
  double computeEffectiveMass(const pinocchio::Model& model, pinocchio::Data& data, 
                              const Eigen::VectorXd& q, const Eigen::Vector3d& u, 
                              const std::string& link_name, const Eigen::Vector3d& point,
                              bool franka_verbose = false, bool verbose = false,
                              Eigen::Matrix<double, 6, 7>* J_out = nullptr);

  Matrix7d MassMatrix(const Vector7d& q);
  Vector7d Friction(const Vector7d& dq);

  rclcpp::Service<multi_mode_control_msgs::srv::JointCollisionGoal>::SharedPtr goal_service_;
  void goalCallback(const std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Request> request,
                    std::shared_ptr<multi_mode_control_msgs::srv::JointCollisionGoal::Response> response);

  const double  FI_11 = 0.54615;
  const double  FI_12 = 0.87224;
  const double  FI_13 = 0.64068;
  const double  FI_14 = 1.2794;
  const double  FI_15 = 0.83904;
  const double  FI_16 = 0.30301;
  const double  FI_17 = 0.56489;

  const double  FI_21 = 5.1181;
  const double  FI_22 = 9.0657;
  const double  FI_23 = 10.136;
  const double  FI_24 = 5.5903;
  const double  FI_25 = 8.3469;
  const double  FI_26 = 17.133;
  const double  FI_27 = 10.336;

  const double  FI_31 = 0.039533;
  const double  FI_32 = 0.025882;
  const double  FI_33 = -0.04607;
  const double  FI_34 = 0.036194;
  const double  FI_35 = 0.026226;
  const double  FI_36 = -0.021047;
  const double  FI_37 = 0.0035526;

  const double TAU_F_CONST_1 = FI_11/(1+exp(-FI_21*FI_31));
  const double TAU_F_CONST_2 = FI_12/(1+exp(-FI_22*FI_32));
  const double TAU_F_CONST_3 = FI_13/(1+exp(-FI_23*FI_33));
  const double TAU_F_CONST_4 = FI_14/(1+exp(-FI_24*FI_34));
  const double TAU_F_CONST_5 = FI_15/(1+exp(-FI_25*FI_35));
  const double TAU_F_CONST_6 = FI_16/(1+exp(-FI_26*FI_36));
  const double TAU_F_CONST_7 = FI_17/(1+exp(-FI_27*FI_37));
};

}  // namespace franka_example_controllers