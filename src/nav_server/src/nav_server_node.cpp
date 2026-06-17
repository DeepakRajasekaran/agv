#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <pqxx/pqxx>
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <algorithm>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

struct Vertex {
    int id;
    double x;
    double y;
    std::string type;
};

struct Edge {
    int id;
    int start_id;
    int end_id;
    bool is_bidirectional;
    std::string curve_type;
    std::vector<std::pair<double, double>> control_points;
};

inline std::vector<std::pair<double, double>> parse_control_points(const std::string& json_str) {
    std::vector<std::pair<double, double>> points;
    if (json_str.empty()) return points;
    std::stringstream ss(json_str);
    char ch;
    double x, y;
    ss >> ch;
    if (ch != '[') return points;
    while (ss >> ch) {
        if (ch == ']') {
            break;
        }
        if (ch == '[') {
            ss >> x >> ch >> y >> ch; // reads x, comma, y, closing bracket
            points.push_back({x, y});
        }
    }
    return points;
}

class NavServerNode : public rclcpp::Node {
public:
    NavServerNode() : Node("nav_server") {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&NavServerNode::handle_goal, this, _1, _2),
            std::bind(&NavServerNode::handle_cancel, this, _1),
            std::bind(&NavServerNode::handle_accepted, this, _1)
        );

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        tag_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sensor/tag_id", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->last_tag_ = msg->data;
            }
        );

        try {
            // Using docker bridge IP to reach the host where Postgres is exposed.
            // Adjust to '127.0.0.1' if running locally without Docker isolation.
            std::string conn_str = "dbname=map_db user=nav_user password=nav_password host=172.18.0.1 port=5432";
            db_conn_ = std::make_unique<pqxx::connection>(conn_str);
            if (db_conn_->is_open()) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected to PostgreSQL map_db.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open map_db.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "DB Connection Error: %s", e.what());
        }

        this->declare_parameter<double>("nominal_speed", 1.5);
        this->get_parameter("nominal_speed", nominal_speed_);

        nav_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);

        RCLCPP_INFO(this->get_logger(), "NavServer Initialized. Nominal speed: %.2f", nominal_speed_);
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tag_sub_;
    std::unique_ptr<pqxx::connection> db_conn_;
    std::string last_tag_ = "";
    double nominal_speed_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request: Target [x: %f, y: %f]", 
            goal->pose.pose.position.x, goal->pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&NavServerNode::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        auto result = std::make_shared<NavigateToPose::Result>();
        auto goal = goal_handle->get_goal();

        // 1. Fetch map from DB
        std::unordered_map<int, Vertex> vertices;
        std::vector<Edge> edges;
        try {
            pqxx::work w(*db_conn_);
            pqxx::result r_v = w.exec("SELECT id, pos_x, pos_y, type FROM vertices;");
            for (auto const &row : r_v) {
                vertices[row[0].as<int>()] = {row[0].as<int>(), row[1].as<double>(), row[2].as<double>(), row[3].as<std::string>()};
            }

            pqxx::result r_e = w.exec("SELECT id, start_vertex_id, end_vertex_id, is_bidirectional, curve_type, control_points FROM edges;");
            for (auto const &row : r_e) {
                std::string cp_str = row[5].is_null() ? "" : row[5].as<std::string>();
                std::vector<std::pair<double, double>> cp = parse_control_points(cp_str);
                edges.push_back({row[0].as<int>(), row[1].as<int>(), row[2].as<int>(), row[3].as<bool>(), row[4].as<std::string>(), cp});
            }
            w.commit();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "DB Fetch Error: %s", e.what());
            goal_handle->abort(result);
            return;
        }

        // 2. Determine Start Vertex
        int start_id = -1;
        if (!last_tag_.empty()) {
            for (const auto& [id, v] : vertices) {
                if (v.type == last_tag_) {
                    start_id = id;
                    break;
                }
            }
        }
        
        if (start_id == -1) {
            // Default to nearest vertex to robot default start (-2.5, 2.02)
            double rx = -2.5, ry = 2.02;
            double min_dist_s = 1e9;
            for (const auto& [id, v] : vertices) {
                double ds = std::hypot(v.x - rx, v.y - ry);
                if (ds < min_dist_s) {
                    min_dist_s = ds;
                    start_id = id;
                }
            }
        }

        // Determine Goal Vertex (nearest to requested pose)
        int goal_id = -1;
        double min_dist_g = 1e9;
        double gx = goal->pose.pose.position.x, gy = goal->pose.pose.position.y;
        for (const auto& [id, v] : vertices) {
            double dg = std::hypot(v.x - gx, v.y - gy);
            if (dg < min_dist_g) {
                min_dist_g = dg;
                goal_id = id;
            }
        }

        if (start_id == -1 || goal_id == -1) {
            RCLCPP_ERROR(this->get_logger(), "No valid vertices found in map!");
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planning path from start vertex ID %d (%s) to goal vertex ID %d (%s)", 
            start_id, vertices[start_id].type.c_str(), goal_id, vertices[goal_id].type.c_str());

        // 3. Dijkstra's Algorithm
        std::unordered_map<int, double> dists;
        std::unordered_map<int, int> previous_nodes;
        for (const auto& [id, v] : vertices) {
            dists[id] = 1e9;
            previous_nodes[id] = -1;
        }
        dists[start_id] = 0.0;

        using pq_elem = std::pair<double, int>;
        std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>> pq;
        pq.push({0.0, start_id});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dists[u]) continue;
            if (u == goal_id) break;

            for (const auto& edge : edges) {
                int v = -1;
                if (edge.start_id == u) {
                    v = edge.end_id;
                } else if (edge.is_bidirectional && edge.end_id == u) {
                    v = edge.start_id;
                }

                if (v != -1) {
                    double weight = std::hypot(vertices[v].x - vertices[u].x, vertices[v].y - vertices[u].y);
                    if (dists[u] + weight < dists[v]) {
                        dists[v] = dists[u] + weight;
                        previous_nodes[v] = u;
                        pq.push({dists[v], v});
                    }
                }
            }
        }

        if (dists[goal_id] > 1e8) {
            RCLCPP_ERROR(this->get_logger(), "No path found from %d to %d!", start_id, goal_id);
            goal_handle->abort(result);
            return;
        }

        // 4. Reconstruct Path
        std::vector<int> path_ids;
        int curr = goal_id;
        while (curr != -1) {
            path_ids.push_back(curr);
            if (curr == start_id) break;
            curr = previous_nodes[curr];
        }
        std::reverse(path_ids.begin(), path_ids.end());

        // Construct path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        std::stringstream ss;
        for (size_t i = 0; i < path_ids.size(); ++i) {
            int u = path_ids[i];
            
            // Add start vertex pose
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = vertices[u].x;
            p.pose.position.y = vertices[u].y;
            p.pose.position.z = 0.0;
            
            // Only add if not duplicate (e.g. overlap from previous edge end)
            if (path_msg.poses.empty() || 
                std::hypot(path_msg.poses.back().pose.position.x - p.pose.position.x, 
                           path_msg.poses.back().pose.position.y - p.pose.position.y) > 0.01) {
                path_msg.poses.push_back(p);
            }
            
            ss << u << " (" << vertices[u].type << ")";
            if (i + 1 < path_ids.size()) {
                ss << " -> ";
                int v = path_ids[i+1];
                // Find edge between u and v
                for (const auto& edge : edges) {
                    if (edge.start_id == u && edge.end_id == v) {
                        // Forward traversal: add control points
                        for (const auto& pt : edge.control_points) {
                            if (path_msg.poses.empty() || 
                                std::hypot(path_msg.poses.back().pose.position.x - pt.first, 
                                           path_msg.poses.back().pose.position.y - pt.second) > 0.01) {
                                geometry_msgs::msg::PoseStamped cp_p;
                                cp_p.pose.position.x = pt.first;
                                cp_p.pose.position.y = pt.second;
                                cp_p.pose.position.z = 0.0;
                                path_msg.poses.push_back(cp_p);
                            }
                        }
                        break;
                    } else if (edge.is_bidirectional && edge.start_id == v && edge.end_id == u) {
                        // Reverse traversal: add control points in reverse
                        for (auto it = edge.control_points.rbegin(); it != edge.control_points.rend(); ++it) {
                            if (path_msg.poses.empty() || 
                                std::hypot(path_msg.poses.back().pose.position.x - it->first, 
                                           path_msg.poses.back().pose.position.y - it->second) > 0.01) {
                                geometry_msgs::msg::PoseStamped cp_p;
                                cp_p.pose.position.x = it->first;
                                cp_p.pose.position.y = it->second;
                                cp_p.pose.position.z = 0.0;
                                path_msg.poses.push_back(cp_p);
                            }
                        }
                        break;
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Reconstructed Path: %s", ss.str().c_str());

        // If we are already at the target, we can succeed immediately
        if (start_id == goal_id) {
            RCLCPP_INFO(this->get_logger(), "Already at the target tag %s. Goal reached.", vertices[goal_id].type.c_str());
            goal_handle->succeed(result);
            return;
        }

        // Publish
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published continuous path waypoints to /plan.");

        std::string target_tag = vertices[goal_id].type;
        RCLCPP_INFO(this->get_logger(), "Waiting for robot to reach target tag: %s", target_tag.c_str());
        
        rclcpp::Rate rate(10.0); // 10Hz
        bool reached = false;
        auto start_time = this->now();
        
        // Reset last tag to ensure we don't match dynamically on stale reads
        this->last_tag_ = "";
        
        while (rclcpp::ok() && !reached) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled by client.");
                geometry_msgs::msg::Twist stop_vel;
                nav_vel_pub_->publish(stop_vel);
                
                nav_msgs::msg::Path empty_path;
                empty_path.header.stamp = this->now();
                empty_path.header.frame_id = "map";
                path_pub_->publish(empty_path);

                goal_handle->canceled(result);
                return;
            }
            
            if (this->last_tag_ == target_tag) {
                reached = true;
                break;
            }
            
            // Timeout if we don't reach in 180 seconds
            if ((this->now() - start_time).seconds() > 180.0) {
                RCLCPP_ERROR(this->get_logger(), "Navigation timeout waiting for tag %s", target_tag.c_str());
                geometry_msgs::msg::Twist stop_vel;
                nav_vel_pub_->publish(stop_vel);
                
                nav_msgs::msg::Path empty_path;
                empty_path.header.stamp = this->now();
                empty_path.header.frame_id = "map";
                path_pub_->publish(empty_path);

                goal_handle->abort(result);
                return;
            }
            
            // Publish current navigation command velocity
            geometry_msgs::msg::Twist nav_vel;
            nav_vel.linear.x = nominal_speed_;
            nav_vel_pub_->publish(nav_vel);
            
            rate.sleep();
        }
        
        if (reached) {
            geometry_msgs::msg::Twist stop_vel;
            nav_vel_pub_->publish(stop_vel);
            
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = this->now();
            empty_path.header.frame_id = "map";
            path_pub_->publish(empty_path);

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded: reached target tag %s.", target_tag.c_str());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
