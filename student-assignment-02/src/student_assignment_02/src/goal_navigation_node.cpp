#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using std::placeholders::_1;

class GoalNavigationNode : public rclcpp::Node {
public:
    GoalNavigationNode() : rclcpp::Node("goal_navigation_node") {
        // Subscriber na planiranu putanju
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path",
            rclcpp::QoS(10),
            std::bind(&GoalNavigationNode::path_callback, this, _1));

        // Publisher za brzinu robota
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Timer za gibanje
        move_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GoalNavigationNode::move_callback, this));

        // Parametri
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("angular_speed", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("angle_tolerance", 0.1);

        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();

        RCLCPP_INFO(this->get_logger(), "Goal Navigation Node inicijaliziran");
        RCLCPP_INFO(this->get_logger(), "  Linear speed: %.2f m/s", linear_speed_);
        RCLCPP_INFO(this->get_logger(), "  Angular speed: %.2f rad/s", angular_speed_);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr move_timer_;

    nav_msgs::msg::Path current_path_;
    bool path_received_ = false;
    size_t current_waypoint_idx_ = 0;
    bool is_moving_ = false;

    double linear_speed_;
    double angular_speed_;
    double goal_tolerance_;
    double angle_tolerance_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;
        path_received_ = true;
        current_waypoint_idx_ = 0;
        is_moving_ = true;

        RCLCPP_INFO(this->get_logger(), "Nova putanja primljena s %zu točaka", 
                   current_path_.poses.size());
    }

    void move_callback() {
        if (!path_received_ || !is_moving_ || current_path_.poses.empty()) {
            // Stani
            stop_robot();
            return;
        }

        // Ako smo dostigli zadnji waypoint
        if (current_waypoint_idx_ >= current_path_.poses.size()) {
            RCLCPP_INFO(this->get_logger(), "Cilj dostignut! Gibanje završeno.");
            stop_robot();
            is_moving_ = false;
            path_received_ = false;
            return;
        }

        // Trenutni cilj
        auto target_pose = current_path_.poses[current_waypoint_idx_].pose;
        auto target_x = target_pose.position.x;
        auto target_y = target_pose.position.y;

        // Simulirana pozicija robota (u stvarnoj situaciji biće iz /tf ili /odom)
        // Za sada pretpostavljamo da se robot giba u smjeru putanje
        static double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
        static bool first_call = true;

        if (first_call) {
            if (!current_path_.poses.empty()) {
                robot_x = current_path_.poses[0].pose.position.x;
                robot_y = current_path_.poses[0].pose.position.y;
                first_call = false;
            }
        }

        // Izračun distancije do cilja
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Ako je distanca manja od tolerance, idi na sljedeći waypoint
        if (distance < goal_tolerance_) {
            current_waypoint_idx_++;
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu dostignut. Ideći...",
                       current_waypoint_idx_);
            return;
        }

        // Izračun kuta do cilja
        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - robot_theta;

        // Normaliziranje kuta na [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        geometry_msgs::msg::Twist cmd_vel;

        // Ako trebamo rotirati
        if (std::abs(angle_diff) > angle_tolerance_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = (angle_diff > 0) ? angular_speed_ : -angular_speed_;
            
            RCLCPP_DEBUG(this->get_logger(), "Rotacija: angle_diff=%.2f rad", angle_diff);
        } else {
            // Kreni prema cilju
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = angular_speed_ * (angle_diff / 0.1);  // PD kontrola

            // Simulacija gibanja robota
            robot_x += linear_speed_ * std::cos(robot_theta) * 0.1;
            robot_y += linear_speed_ * std::sin(robot_theta) * 0.1;
            robot_theta += cmd_vel.angular.z * 0.1;

            if (current_waypoint_idx_ % 5 == 0) {
                RCLCPP_DEBUG(this->get_logger(), 
                           "Gibanje: robot(%.2f, %.2f) -> target(%.2f, %.2f), d=%.2f",
                           robot_x, robot_y, target_x, target_y, distance);
            }
        }

        cmd_vel_publisher_->publish(cmd_vel);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
