#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <random>
#include <vector>
#include <string>
#include <map>
#include <algorithm>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "effective_mass_stats_vis/mass_matrix.hpp"

class GenericPointEffectiveMassVisualization : public rclcpp::Node
{
public:
    struct SampledPoint {
        geometry_msgs::msg::Point position;
        Eigen::Vector3d normal;
        double m_eff;
    };

    GenericPointEffectiveMassVisualization() : Node("generic_point_effective_mass_visualization")
    {
        this->declare_parameter<std::vector<double>>("impact_direction", {1.0, 0.0, 0.0});

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/effective_mass_points", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&GenericPointEffectiveMassVisualization::jointStateCallback, this, std::placeholders::_1));
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&GenericPointEffectiveMassVisualization::clickedPointCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GenericPointEffectiveMassVisualization::publishMarkers, this));

        std::string package_share_directory;
        try {
            package_share_directory = ament_index_cpp::get_package_share_directory("franka_description");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get franka_description package share directory: %s", e.what());
            return;
        }

        std::string urdf_path = package_share_directory + "/robots/real/panda_arm.urdf";
        pinocchio::urdf::buildModel(urdf_path, model_pin_);
        data_pin_ = pinocchio::Data(model_pin_);
        
        q_curr_.resize(7);
        q_curr_.setZero();
        has_joint_states_ = false;

        std::vector<std::string> link_names = {
            "link0", "link1", "link2", "link3", 
            "link4", "link5", "link6", "link7"
        };

        const int num_points_per_link = 1000;

        for (const auto& link_name : link_names) {
            std::string mesh_path = package_share_directory + "/meshes/visual/" + link_name + ".dae";
            std::vector<SampledPoint> sampled_points;
            
            samplePointsFromMesh(mesh_path, num_points_per_link, sampled_points);

            if (!sampled_points.empty()) {
                std::string frame_id = "panda_" + link_name;
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = frame_id;
                marker.ns = "effective_mass_points";
                marker.id = marker_array_.markers.size();
                marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.01;
                marker.scale.y = 0.01;
                marker.scale.z = 0.01;

                for (const auto& sp : sampled_points) {
                    marker.points.push_back(sp.position);
                    std_msgs::msg::ColorRGBA color;
                    color.r = 0.0;
                    color.g = 1.0;
                    color.b = 0.0;
                    color.a = 1.0;
                    marker.colors.push_back(color);
                }

                marker_array_.markers.push_back(marker);
                link_sampled_points_[frame_id] = sampled_points;
                RCLCPP_INFO(this->get_logger(), "Sampled %d points for %s", num_points_per_link, frame_id.c_str());
            }
        }
    }

private:
    void samplePointsFromMesh(const std::string& filename, int num_points, std::vector<SampledPoint>& points)
    {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
        if (!scene || !scene->HasMeshes()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mesh: %s", filename.c_str());
            return;
        }

        std::vector<double> areas;
        std::vector<const aiFace*> all_faces;
        std::vector<const aiMesh*> all_meshes;
        std::vector<aiMatrix4x4> all_transforms;

        std::function<void(const aiNode*, aiMatrix4x4, bool)> collectFaces = [&](const aiNode* node, aiMatrix4x4 parent_transform, bool is_root) {
            aiMatrix4x4 transform;
            if (is_root) {
                aiVector3D scaling, position;
                aiQuaternion rotation;
                node->mTransformation.Decompose(scaling, rotation, position);
                
                aiMatrix4x4 scale_matrix;
                aiMatrix4x4::Scaling(scaling, scale_matrix);
                transform = scale_matrix;
            } else {
                transform = parent_transform * node->mTransformation;
            }

            for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
                const aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
                for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
                    const aiFace& face = mesh->mFaces[f];
                    if (face.mNumIndices == 3) {
                        aiVector3D v0 = transform * mesh->mVertices[face.mIndices[0]];
                        aiVector3D v1 = transform * mesh->mVertices[face.mIndices[1]];
                        aiVector3D v2 = transform * mesh->mVertices[face.mIndices[2]];
                        
                        double cx = (v1.y - v0.y) * (v2.z - v0.z) - (v1.z - v0.z) * (v2.y - v0.y);
                        double cy = (v1.z - v0.z) * (v2.x - v0.x) - (v1.x - v0.x) * (v2.z - v0.z);
                        double cz = (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
                        double area = 0.5 * std::sqrt(cx*cx + cy*cy + cz*cz);
                        
                        if (area > 1e-9) {
                            areas.push_back(area);
                            all_faces.push_back(&face);
                            all_meshes.push_back(mesh);
                            all_transforms.push_back(transform);
                        }
                    }
                }
            }
            for (unsigned int i = 0; i < node->mNumChildren; ++i) {
                collectFaces(node->mChildren[i], transform, false);
            }
        };

        collectFaces(scene->mRootNode, aiMatrix4x4(), true);

        if (areas.empty()) {
            RCLCPP_WARN(this->get_logger(), "Mesh has no valid triangles: %s", filename.c_str());
            return;
        }

        double total_area = 0.0;
        for (double a : areas) total_area += a;
        
        // Estimate ideal distance between points assuming uniform distribution
        double density = num_points / total_area;
        double min_dist = 0.6 * std::sqrt(1.0 / density); 
        double min_dist_sq = min_dist * min_dist;

        std::discrete_distribution<> dist(areas.begin(), areas.end());
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<> rnd(0.0, 1.0);

        int max_candidates = num_points * 50; 
        for (int i = 0; i < max_candidates; ++i) {
            int face_idx = dist(gen);
            const aiFace* face = all_faces[face_idx];
            const aiMesh* mesh = all_meshes[face_idx];
            aiMatrix4x4 transform = all_transforms[face_idx];

            aiVector3D v0 = transform * mesh->mVertices[face->mIndices[0]];
            aiVector3D v1 = transform * mesh->mVertices[face->mIndices[1]];
            aiVector3D v2 = transform * mesh->mVertices[face->mIndices[2]];

            double r1 = std::sqrt(rnd(gen));
            double r2 = rnd(gen);
            double u = 1.0 - r1;
            double v = r1 * (1.0 - r2);
            double w = 1.0 - u - v;

            aiMatrix3x3 t_inv_3x3(transform);
            t_inv_3x3.Inverse();
            t_inv_3x3.Transpose();

            aiVector3D n0, n1, n2;
            if (mesh->HasNormals()) {
                n0 = mesh->mNormals[face->mIndices[0]];
                n1 = mesh->mNormals[face->mIndices[1]];
                n2 = mesh->mNormals[face->mIndices[2]];
            } else {
                aiVector3D e1 = v1 - v0;
                aiVector3D e2 = v2 - v0;
                n0 = n1 = n2 = (e1 ^ e2).Normalize();
            }

            aiVector3D n0_trans = t_inv_3x3 * n0;
            aiVector3D n1_trans = t_inv_3x3 * n1;
            aiVector3D n2_trans = t_inv_3x3 * n2;
            
            aiVector3D n = (float)u * n0_trans + (float)v * n1_trans + (float)w * n2_trans;
            n.Normalize();

            SampledPoint sp;
            sp.position.x = u * v0.x + v * v1.x + w * v2.x;
            sp.position.y = u * v0.y + v * v1.y + w * v2.y;
            sp.position.z = u * v0.z + v * v1.z + w * v2.z;
            sp.normal = Eigen::Vector3d(n.x, n.y, n.z);
            sp.m_eff = 0.0;
            
            // Dart-throwing filtering for uniform distribution
            bool ok = true;
            for (const auto& fp : points) {
                double d2 = (sp.position.x-fp.position.x)*(sp.position.x-fp.position.x) + 
                            (sp.position.y-fp.position.y)*(sp.position.y-fp.position.y) + 
                            (sp.position.z-fp.position.z)*(sp.position.z-fp.position.z);
                if (d2 < min_dist_sq) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                points.push_back(sp);
                if (points.size() >= (size_t)num_points) break;
            }
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string name = msg->name[i];
            if (name.find("panda_joint") != std::string::npos && name.size() > 11) {
                int joint_idx = name[11] - '1';
                if (joint_idx >= 0 && joint_idx < 7) {
                    q_curr_(joint_idx) = msg->position[i];
                    has_joint_states_ = true;
                }
            }
        }
    }

    void publishMarkers()
    {
        if (!has_joint_states_ || marker_array_.markers.empty()) return;

        std::vector<double> u_d_vec;
        this->get_parameter("impact_direction", u_d_vec);
        Eigen::Vector3d u(u_d_vec[0], u_d_vec[1], u_d_vec[2]);
        if (u.norm() < 1e-6) u << 1.0, 0.0, 0.0;
        else u.normalize();

        // 1. Calculate Analytical Mass Matrix
        Vector7d q_7d = q_curr_;
        Matrix7d M_gaz = MassMatrix(q_7d);
        Matrix7d M_gaz_inv = M_gaz.inverse();

        // 2. Pinocchio FK
        pinocchio::framesForwardKinematics(model_pin_, data_pin_, q_curr_);
        
        for (auto& marker : marker_array_.markers) {
            marker.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            std::string frame_id = marker.header.frame_id;
            if (model_pin_.existFrame(frame_id)) {
                auto pin_frame_id = model_pin_.getFrameId(frame_id);
                Eigen::Matrix<double, 6, 7> J_pin_ee;
                J_pin_ee.setZero();
                pinocchio::computeFrameJacobian(model_pin_, data_pin_, q_curr_, pin_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_pin_ee);

                Eigen::Matrix3d R = data_pin_.oMf[pin_frame_id].rotation();

                std::vector<SampledPoint>& sampled_points = link_sampled_points_[frame_id];

                for (size_t i = 0; i < marker.points.size(); ++i) {
                    auto pt = marker.points[i];
                    Eigen::Vector3d pt_eigen(pt.x, pt.y, pt.z);
                    Eigen::Vector3d p_world = R * pt_eigen;
                    
                    Eigen::Matrix3d p_cross;
                    p_cross << 0, -p_world(2), p_world(1),
                               p_world(2), 0, -p_world(0),
                               -p_world(1), p_world(0), 0;

                    Eigen::Matrix<double, 6, 7> J_pin_pt = J_pin_ee;
                    J_pin_pt.topRows<3>() -= p_cross * J_pin_pt.bottomRows<3>();

                    Eigen::Matrix<double, 6, 6> JMJt_gaz_pt = J_pin_pt * M_gaz_inv * J_pin_pt.transpose();
                    double m_eff = 1.0 / (u.transpose() * (JMJt_gaz_pt.block<3,3>(0,0)) * u);

                    if (i < sampled_points.size()) {
                        sampled_points[i].m_eff = m_eff;
                    }

                    double normalized = std::clamp((m_eff - 0.0) / 20.0, 0.0, 1.0);
                    marker.colors[i].r = normalized;
                    marker.colors[i].g = 1.0 - normalized;
                    marker.colors[i].b = 0.0;
                }
            }
        }
        
        marker_pub_->publish(marker_array_);
    }

    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!has_joint_states_) return;
        
        Eigen::Vector3d clicked_p(msg->point.x, msg->point.y, msg->point.z);
        
        double min_dist_sq = std::numeric_limits<double>::max();
        std::string best_link = "";
        size_t best_idx = 0;
        Eigen::Vector3d best_point_world;
        
        for (const auto& kv : link_sampled_points_) {
            std::string frame_id = kv.first;
            if (!model_pin_.existFrame(frame_id)) continue;
            auto pin_frame_id = model_pin_.getFrameId(frame_id);
            const pinocchio::SE3& oMf = data_pin_.oMf[pin_frame_id];
            
            for (size_t i = 0; i < kv.second.size(); ++i) {
                const auto& sp = kv.second[i];
                Eigen::Vector3d pt_local(sp.position.x, sp.position.y, sp.position.z);
                Eigen::Vector3d pt_world = oMf.act(pt_local); 
                
                double dist_sq = (pt_world - clicked_p).squaredNorm();
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_link = frame_id;
                    best_idx = i;
                    best_point_world = pt_world;
                }
            }
        }
        
        if (best_link != "") {
            const auto& sp = link_sampled_points_[best_link][best_idx];
            auto pin_frame_id = model_pin_.getFrameId(best_link);
            const pinocchio::SE3& oMf = data_pin_.oMf[pin_frame_id];
            
            Eigen::Vector3d normal_local = sp.normal;
            Eigen::Vector3d normal_world = oMf.rotation() * normal_local;
            
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "Clicked Point Matched!");
            RCLCPP_INFO(this->get_logger(), "Link: %s", best_link.c_str());
            RCLCPP_INFO(this->get_logger(), "Local Position: [%.4f, %.4f, %.4f]", sp.position.x, sp.position.y, sp.position.z);
            RCLCPP_INFO(this->get_logger(), "Effective Mass: %.4f kg", sp.m_eff);
            RCLCPP_INFO(this->get_logger(), "Local Normal:   [%.4f, %.4f, %.4f]", normal_local.x(), normal_local.y(), normal_local.z());
            RCLCPP_INFO(this->get_logger(), "Base Normal:    [%.4f, %.4f, %.4f]", normal_world.x(), normal_world.y(), normal_world.z());
            RCLCPP_INFO(this->get_logger(), "Distance to click: %.4f m", std::sqrt(min_dist_sq));
            RCLCPP_INFO(this->get_logger(), "=========================================");
        }
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    visualization_msgs::msg::MarkerArray marker_array_;

    pinocchio::Model model_pin_;
    pinocchio::Data data_pin_;
    Eigen::VectorXd q_curr_;
    bool has_joint_states_{false};

    std::map<std::string, std::vector<SampledPoint>> link_sampled_points_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericPointEffectiveMassVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
