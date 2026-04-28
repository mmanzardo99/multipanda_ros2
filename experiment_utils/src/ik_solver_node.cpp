#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <multi_mode_control_msgs/srv/solve_ik.hpp>
#include <experiment_utils/franka_ik_He.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode() : Node("ik_solver_node") {
        this->declare_parameter<std::string>("robot_description_path", "");
        std::string urdf_path = this->get_parameter("robot_description_path").as_string();

        if (urdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description_path' is empty!");
        } else {
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
            RCLCPP_INFO(this->get_logger(), "Loaded Pinocchio model from %s", urdf_path.c_str());
        }

        service_ = this->create_service<multi_mode_control_msgs::srv::SolveIK>(
            "solve_ik",
            std::bind(&IKSolverNode::solve_ik_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "IK Solver Service ready.");
    }

private:
    void solve_ik_callback(
        const std::shared_ptr<multi_mode_control_msgs::srv::SolveIK::Request> request,
        std::shared_ptr<multi_mode_control_msgs::srv::SolveIK::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "Received IK request.");

        // 1. Position IK
        Eigen::Quaterniond q(
            request->target_pose.orientation.w,
            request->target_pose.orientation.x,
            request->target_pose.orientation.y,
            request->target_pose.orientation.z
        );
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d p(
            request->target_pose.position.x,
            request->target_pose.position.y,
            request->target_pose.position.z
        );

        std::array<double, 16> O_T_EE_array;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = p;
        
        // Eigen is column-major by default, franka_ik_He expects array
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                O_T_EE_array[i + j*4] = T(i,j);

        double q7 = request->q7;
        std::array<double, 7> q_actual_array = {{0, 0, 0, 0, 0, 0, 0}}; // Seed for case consistency

        auto ik_solutions = franka_IK_EE(O_T_EE_array, q7, q_actual_array);

        int best_idx = -1;
        for(int i=0; i<4; ++i) {
            bool valid = true;
            for(int j=0; j<7; ++j) {
                if(std::isnan(ik_solutions[i][j])) {
                    valid = false;
                    break;
                }
            }
            if(valid) {
                best_idx = i;
                break;
            }
        }

        if (best_idx == -1) {
            response->success = false;
            response->message = "No valid IK solution found.";
            RCLCPP_WARN(this->get_logger(), "No valid IK solution found.");
            return;
        }

        Eigen::VectorXd q_sol(7);
        for(int i=0; i<7; ++i) q_sol(i) = ik_solutions[best_idx][i];

        // 2. Velocity IK (Weighted Pseudoinverse)
        // x_dot = [v; omega]
        Eigen::Matrix<double, 6, 1> x_dot;
        x_dot << request->cartesian_velocity.linear.x,
                 request->cartesian_velocity.linear.y,
                 request->cartesian_velocity.linear.z,
                 request->cartesian_velocity.angular.x,
                 request->cartesian_velocity.angular.y,
                 request->cartesian_velocity.angular.z;

        // Compute Jacobian and Mass Matrix
        pinocchio::computeJointJacobians(model_, data_, q_sol);
        pinocchio::framesForwardKinematics(model_, data_, q_sol);
        
        // We assume the EE is the last frame or a specific frame. 
        // For Panda, usually "panda_link8" or "panda_hand".
        std::string ee_frame = "panda_link8"; 
        auto frame_id = model_.getFrameId(ee_frame);
        if (frame_id >= model_.frames.size()) {
             ee_frame = model_.frames.back().name;
             frame_id = model_.getFrameId(ee_frame);
        }

        Eigen::Matrix<double, 6, 7> J;
        J.setZero();
        pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

        Eigen::Matrix<double, 7, 7> W = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 6, 6> JJt = J * W.inverse() * J.transpose();
        Eigen::Matrix<double, 7, 6> J_weighted_pinv = W.inverse() * J.transpose() * JJt.inverse();

        Eigen::VectorXd dq_sol = J_weighted_pinv * x_dot;

        // Populate response
        for(int i=0; i<7; ++i) {
            response->joint_positions.push_back(q_sol(i));
            response->joint_velocities.push_back(dq_sol(i));
        }
        response->success = true;
        response->message = "IK and Velocity solved successfully.";

        // Print info
        std::cout << "--- IK SOLVER INFO ---" << std::endl;
        std::cout << "Target Pose: p=[" << p.transpose() << "]" << std::endl;
        std::cout << "Target q7: " << q7 << std::endl;
        std::cout << "Cartesian Velocity: [" << x_dot.transpose() << "]" << std::endl;
        std::cout << "Joint Positions (rad): " << q_sol.transpose() << std::endl;
        std::cout << "Joint Velocities (rad/s): " << dq_sol.transpose() << std::endl;
        std::cout << "-----------------------" << std::endl;
    }

    rclcpp::Service<multi_mode_control_msgs::srv::SolveIK>::SharedPtr service_;
    pinocchio::Model model_;
    pinocchio::Data data_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
