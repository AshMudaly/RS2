#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <chrono>

class UR3eController : public rclcpp::Node {
public:
    UR3eController() : Node("ur3e_controller") {
        this->declare_parameter("velocity_scaling", 0.1);
        this->get_parameter("velocity_scaling", velocity_scaling_);

        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/ur_status", 10);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_group/goal", 10,
            std::bind(&UR3eController::poseCallback, this, std::placeholders::_1)
        );  
    }

    void initializeMoveGroup() {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::static_pointer_cast<rclcpp::Node>(shared_from_this()), "ur_manipulator"
        );
        move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling_);
        startupInitPose();

        // Add collision objects to the scene
        std::vector<std::vector<double>> objects = {
            {0.0, -0.35, 0.6, 0.0, 0.0, 0.0, 1.0, 1.2, 0.001, 1.2},  // Back Wall
            {0.0, 0.0, -0.005, 0.0, 0.0, 0.0, 1.0, 1.2, 1.2, 0.001} // Floor
        };
        addCollisionObjects(objects);

        RCLCPP_INFO(this->get_logger(), "Publishing ready status.");
        std_msgs::msg::Int32 ready_msg;
        ready_msg.data = 0; // Ready
        status_pub_->publish(ready_msg);
    }

private:
    void startupInitPose() {
        std::vector<double> init_joint_positions = {2.61799, -0.7854, 1.0472, -1.5708, -1.5708, 0.0};
        move_group_interface_->setJointValueTarget(init_joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_interface_->plan(plan)) {
            RCLCPP_INFO(this->get_logger(), "Executing startup pose.");
            move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan startup pose.");
        }
    }

    void scaleTrajectorySpeed(moveit_msgs::msg::RobotTrajectory &trajectory, double scale) {
        for (auto &point : trajectory.joint_trajectory.points) {
            for (auto &vel : point.velocities) vel *= scale;
            for (auto &acc : point.accelerations) acc *= scale;

            int64_t total_ns = static_cast<int64_t>(
                point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            );
            int64_t scaled_ns = static_cast<int64_t>(total_ns / scale);
            point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(scaled_ns));
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!move_group_interface_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received new target pose.");

        std_msgs::msg::Int32 status_msg;
        status_msg.data = 1;  // Executing
        status_pub_->publish(status_msg);

        move_group_interface_->setStartStateToCurrentState();

        std::vector<geometry_msgs::msg::Pose> waypoints = {msg->pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Singularity or planning failed (%.2f%%)", fraction * 100.0);
            status_msg.data = 3;  // Singularity
        } else {
            scaleTrajectorySpeed(trajectory, velocity_scaling_);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            auto result = move_group_interface_->execute(plan);
            status_msg.data = (result == moveit::core::MoveItErrorCode::SUCCESS) ? 2 : 4; // 2 = Done, 4 = Failed
        }

        status_pub_->publish(status_msg);
    }

    void addCollisionObjects(const std::vector<std::vector<double>> &object_specs) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        for (size_t i = 0; i < object_specs.size(); ++i) {
            const auto &spec = object_specs[i];
            if (spec.size() != 10) continue;

            moveit_msgs::msg::CollisionObject object;
            object.header.frame_id = "base_link";
            object.id = "object_" + std::to_string(i);

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {spec[7], spec[8], spec[9]};

            geometry_msgs::msg::Pose pose;
            pose.position.x = spec[0];
            pose.position.y = spec[1];
            pose.position.z = spec[2];
            pose.orientation.x = spec[3];
            pose.orientation.y = spec[4];
            pose.orientation.z = spec[5];
            pose.orientation.w = spec[6];

            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(pose);
            object.operation = object.ADD;

            collision_objects.push_back(object);
        }

        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Added %zu collision objects.", collision_objects.size());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    double velocity_scaling_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3eController>();
    node->initializeMoveGroup();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
