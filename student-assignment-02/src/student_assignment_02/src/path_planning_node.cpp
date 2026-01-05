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
#include <set>

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

    bool operator==(const PathNode& other) const {
        return x == other.x && y == other.y;
    }
};

class PathPlanningNode : public rclcpp::Node {
public:
    PathPlanningNode() : rclcpp::Node("path_planning_node") {
        // NE deklariramo use_sim_time jer je već deklariran od strane launch fajla
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

        // Publisher za vizualizaciju A* pretrage
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", 10);

        // Timer za planiranje
        plan_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PathPlanningNode::plan_path_callback, this));

        RCLCPP_INFO(this->get_logger(), "Path Planning Node inicijaliziran");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr plan_timer_;
    
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;
    int plan_count_ = 0;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        map_received_ = true;
        
        if (plan_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Mapa primljena: %d x %d, rezolucija: %.2f m/cell",
                        current_map_.info.width, current_map_.info.height,
                        current_map_.info.resolution);
        }
    }

    void plan_path_callback() {
        if (!map_received_) return;

        plan_count_++;
        
        // Početna pozicija (lijeva strana)
        int start_x = 5;
        int start_y = current_map_.info.height / 2;
        
        // Ciljna pozicija (desna strana)
        int goal_x = current_map_.info.width - 5;
        int goal_y = current_map_.info.height / 2;

        RCLCPP_INFO(this->get_logger(), "[Plan %d] Planiranje putanje od (%d, %d) do (%d, %d)",
                    plan_count_, start_x, start_y, goal_x, goal_y);

        auto path = a_star(start_x, start_y, goal_x, goal_y);

        if (!path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Putanja pronađena! Duljina: %zu čvorova", path.size());
            
            // Vizualizacija
            visualize_path(path, start_x, start_y, goal_x, goal_y);
        } else {
            RCLCPP_WARN(this->get_logger(), "Nije moguće planirati putanju");
        }
    }

    // Manhattan distanca kao heuristika
    float heuristic(int x1, int y1, int x2, int y2) const {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }

    // Euklidska distanca (alternativa)
    float heuristic_euclidean(int x1, int y1, int x2, int y2) const {
        float dx = x1 - x2;
        float dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
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
        
        // < 50 = slobodno, >= 50 = zauzeto
        return current_map_.data[index] < 50;
    }

    // Pronalaženje čvora u listi
    std::shared_ptr<PathNode> find_node(std::vector<std::shared_ptr<PathNode>>& list, int x, int y) const {
        for (auto& n : list) {
            if (n->x == x && n->y == y) return n;
        }
        return nullptr;
    }

    // A* algoritam sa poboljšanjima
    std::vector<std::pair<int, int>> a_star(int start_x, int start_y, int goal_x, int goal_y) {
        std::vector<std::pair<int, int>> path;
        
        // Ako je cilj nedostižan odmah, vrati praznu putanju
        if (!is_valid(goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "Ciljna pozicija nije dostupna");
            return path;
        }

        std::vector<std::shared_ptr<PathNode>> open_list;
        std::vector<std::shared_ptr<PathNode>> closed_list;
        std::set<std::pair<int, int>> closed_set;  // Za brže pretraživanje

        // Inicijalizacija početnog čvora
        auto start = std::make_shared<PathNode>(start_x, start_y, 0.0f,
                                                heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push_back(start);

        // 8-smjerna kretanja (N, NE, E, SE, S, SW, W, NW)
        int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
        int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};
        float cost[] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};  // Dijagonalne su skuplje

        int iterations = 0;
        const int MAX_ITERATIONS = 100000;

        while (!open_list.empty() && iterations < MAX_ITERATIONS) {
            iterations++;

            // Pronalazi čvor sa najmanjom f_cost
            int current_idx = 0;
            for (int i = 1; i < (int)open_list.size(); i++) {
                if (open_list[i]->f_cost < open_list[current_idx]->f_cost) {
                    current_idx = i;
                }
            }

            auto current = open_list[current_idx];
            open_list.erase(open_list.begin() + current_idx);
            closed_list.push_back(current);
            closed_set.insert({current->x, current->y});

            // Provjerava je li cilj dostignut
            if (current->x == goal_x && current->y == goal_y) {
                RCLCPP_INFO(this->get_logger(), "A* završen u %d iteracija", iterations);
                
                // Rekonstruira putanju
                auto node = current;
                while (node != nullptr) {
                    path.insert(path.begin(), std::make_pair(node->x, node->y));
                    node = node->parent;
                }
                return path;
            }

            // Provjeri sve susjede (8 smjerova)
            for (int i = 0; i < 8; i++) {
                int new_x = current->x + dx[i];
                int new_y = current->y + dy[i];

                // Provjeri je li dostupan
                if (!is_valid(new_x, new_y)) continue;
                
                // Ako je već u closed listi, preskoči
                if (closed_set.count({new_x, new_y})) continue;

                float new_g = current->g_cost + cost[i];
                float new_h = heuristic(new_x, new_y, goal_x, goal_y);

                // Provjeri je li čvor već u open listi
                auto existing = find_node(open_list, new_x, new_y);
                
                if (existing) {
                    // Ako je novi put jeftiniji, ažuriraj
                    if (new_g < existing->g_cost) {
                        existing->g_cost = new_g;
                        existing->f_cost = new_g + existing->h_cost;
                        existing->parent = current;
                    }
                } else {
                    // Dodaj novi čvor u open listu
                    auto new_node = std::make_shared<PathNode>(new_x, new_y, new_g, new_h, current);
                    open_list.push_back(new_node);
                }
            }
        }

        if (iterations >= MAX_ITERATIONS) {
            RCLCPP_WARN(this->get_logger(), "A* dosegao max iteracija (%d)", MAX_ITERATIONS);
        } else {
            RCLCPP_WARN(this->get_logger(), "Putanja nije pronađena nakon %d iteracija", iterations);
        }
        return path;
    }

    void visualize_path(const std::vector<std::pair<int, int>>& path,
                       int start_x, int start_y, int goal_x, int goal_y) {
        visualization_msgs::msg::MarkerArray markers;

        // 1. Vizualizacija putanje (zelena linija)
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = current_map_.header.frame_id;
        path_marker.header.stamp = this->now();
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1;  // Debljina linije
        path_marker.color.r = 0.0f;
        path_marker.color.g = 1.0f;
        path_marker.color.b = 0.0f;
        path_marker.color.a = 1.0f;
        path_marker.pose.orientation.w = 1.0;

        for (const auto& p : path) {
            geometry_msgs::msg::Point pt;
            pt.x = (p.first + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
            pt.y = (p.second + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
            pt.z = 0.05;  // Malo iznad mape
            path_marker.points.push_back(pt);
        }

        markers.markers.push_back(path_marker);

        // 2. Početna točka (zelena sfera)
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
        start_marker.pose.position.x = (start_x + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
        start_marker.pose.position.y = (start_y + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
        start_marker.pose.position.z = 0.0;
        start_marker.pose.orientation.w = 1.0;

        markers.markers.push_back(start_marker);

        // 3. Ciljna točka (crvena sfera)
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
        goal_marker.pose.position.x = (goal_x + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.x;
        goal_marker.pose.position.y = (goal_y + 0.5f) * current_map_.info.resolution + current_map_.info.origin.position.y;
        goal_marker.pose.position.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;

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
