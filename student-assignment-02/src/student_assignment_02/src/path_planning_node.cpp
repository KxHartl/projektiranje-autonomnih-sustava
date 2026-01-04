#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

using std::placeholders::_1;

// Struktura za čvor A* algoritma
struct PathNode {
    int x, y;
    float g_cost;  // Cijena od početka
    float h_cost;  // Heuristička procjena do cilja
    float f_cost;  // g_cost + h_cost
    std::shared_ptr<PathNode> parent;

    PathNode(int x, int y, float g, float h, std::shared_ptr<PathNode> p = nullptr)
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    bool operator>(const PathNode& other) const {
        return f_cost > other.f_cost;
    }
};

class PathPlanningNode : public rclcpp::Node {
public:
    PathPlanningNode() : rclcpp::Node("path_planning_node") {
        // NE deklariramo use_sim_time jer je već deklariran od strane launch fajla
        // Samo čitamo vrijednost ako postoji
        try {
            auto use_sim_time = this->get_parameter_or<bool>("use_sim_time", false);
            RCLCPP_INFO(this->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Could not read use_sim_time parameter");
        }

        // Subscriber na mapu
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::SensorDataQoS(),
            std::bind(&PathPlanningNode::map_callback, this, _1));

        // Publisher za vizualizaciju pretrage
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", 10);

        RCLCPP_INFO(this->get_logger(), "Path Planning Node inicijaliziran");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Mapa primljena: %d x %d", 
                    current_map_.info.width, current_map_.info.height);

        // Primjer: Planiranje putanje od (1, 1) do (10, 10)
        if (map_received_) {
            plan_path(1, 1, 10, 10);
        }
    }

    // Manhattan distanca kao heuristika
    float heuristic(int x1, int y1, int x2, int y2) const {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }

    // Provjera je li čvor dostupan (nije zauzet)
    bool is_valid(int x, int y) const {
        if (x < 0 || x >= (int)current_map_.info.width ||
            y < 0 || y >= (int)current_map_.info.height) {
            return false;
        }
        
        int index = y * current_map_.info.width + x;
        if (index < 0 || index >= (int)current_map_.data.size()) {
            return false;
        }
        
        return current_map_.data[index] < 50;  // <50 = slobodno
    }

    // Provjera je li čvor već bio otvoren
    bool node_in_list(const std::vector<std::shared_ptr<PathNode>>& list, int x, int y) const {
        return std::any_of(list.begin(), list.end(),
            [x, y](const std::shared_ptr<PathNode>& n) { return n->x == x && n->y == y; });
    }

    // Pronalaženje čvora u listi
    std::shared_ptr<PathNode> find_node(std::vector<std::shared_ptr<PathNode>>& list, int x, int y) const {
        for (auto& n : list) {
            if (n->x == x && n->y == y) return n;
        }
        return nullptr;
    }

    // A* algoritam
    std::vector<std::pair<int, int>> a_star(int start_x, int start_y, int goal_x, int goal_y) {
        std::vector<std::pair<int, int>> path;
        std::vector<std::shared_ptr<PathNode>> open_list;
        std::vector<std::shared_ptr<PathNode>> closed_list;

        // Inicijalizacija
        auto start = std::make_shared<PathNode>(start_x, start_y, 0, 
                                            heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push_back(start);

        // Smjerovi kretanja (8 smjerova)
        int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

        while (!open_list.empty()) {
            // Pronalazi čvor sa najmanjom f cost
            int current_idx = 0;
            for (int i = 1; i < (int)open_list.size(); i++) {
                if (open_list[i]->f_cost < open_list[current_idx]->f_cost) {
                    current_idx = i;
                }
            }

            auto current = open_list[current_idx];
            open_list.erase(open_list.begin() + current_idx);
            closed_list.push_back(current);

            // Provjerava je li cilj dostignut
            if (current->x == goal_x && current->y == goal_y) {
                // Rekonstruira putanju
                auto node = current;
                while (node != nullptr) {
                    path.insert(path.begin(), std::make_pair(node->x, node->y));
                    node = node->parent;
                }
                return path;
            }

            // Provjeri sve susjede
            for (int i = 0; i < 8; i++) {
                int new_x = current->x + dx[i];
                int new_y = current->y + dy[i];

                if (!is_valid(new_x, new_y)) continue;
                if (node_in_list(closed_list, new_x, new_y)) continue;

                float new_g = current->g_cost + std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
                float new_h = heuristic(new_x, new_y, goal_x, goal_y);

                auto existing = find_node(open_list, new_x, new_y);
                if (existing && new_g >= existing->g_cost) continue;

                auto new_node = std::make_shared<PathNode>(new_x, new_y, new_g, new_h, current);
                if (!existing) {
                    open_list.push_back(new_node);
                } else {
                    existing->g_cost = new_g;
                    existing->f_cost = new_g + existing->h_cost;
                    existing->parent = current;
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "Putanja nije pronađena!");
        return path;
    }

    void plan_path(int start_x, int start_y, int goal_x, int goal_y) {
        if (!map_received_) return;

        RCLCPP_INFO(this->get_logger(), "Planiranje putanje od (%d, %d) do (%d, %d)",
                    start_x, start_y, goal_x, goal_y);

        auto path = a_star(start_x, start_y, goal_x, goal_y);

        if (!path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Putanja pronađena! Duljina: %zu", path.size());
            visualize_path(path);
        } else {
            RCLCPP_WARN(this->get_logger(), "Nije moguće planirati putanju");
        }
    }

    void visualize_path(const std::vector<std::pair<int, int>>& path) {
        visualization_msgs::msg::MarkerArray markers;

        // Vizualizacija putanje
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = current_map_.header.frame_id;
        path_marker.header.stamp = this->now();
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.05;
        path_marker.color.r = 0.0f;
        path_marker.color.g = 1.0f;
        path_marker.color.b = 0.0f;
        path_marker.color.a = 1.0f;

        for (const auto& p : path) {
            geometry_msgs::msg::Point pt;
            pt.x = (p.first + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
            pt.y = (p.second + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
            pt.z = 0.0;
            path_marker.points.push_back(pt);
        }

        markers.markers.push_back(path_marker);

        // Vizualizacija početne točke
        visualization_msgs::msg::Marker start_marker;
        start_marker.header = path_marker.header;
        start_marker.ns = "start";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        start_marker.color.r = 0.0f;
        start_marker.color.g = 1.0f;
        start_marker.color.b = 0.0f;
        start_marker.color.a = 1.0f;
        start_marker.pose.position.x = (path[0].first + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
        start_marker.pose.position.y = (path[0].second + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
        start_marker.pose.position.z = 0.0;

        markers.markers.push_back(start_marker);

        // Vizualizacija ciljne točke
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header = path_marker.header;
        goal_marker.ns = "goal";
        goal_marker.id = 2;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.scale.x = 0.3;
        goal_marker.scale.y = 0.3;
        goal_marker.scale.z = 0.3;
        goal_marker.color.r = 1.0f;
        goal_marker.color.g = 0.0f;
        goal_marker.color.b = 0.0f;
        goal_marker.color.a = 1.0f;
        goal_marker.pose.position.x = (path.back().first + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
        goal_marker.pose.position.y = (path.back().second + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
        goal_marker.pose.position.z = 0.0;

        markers.markers.push_back(goal_marker);

        marker_publisher_->publish(markers);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}
