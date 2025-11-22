/**********************************************************************
 * GO2 Waypoint Navigator with CSV
 * - ROS2 Foxy
 * - CSV 파일에서 웨이포인트 로드 (x,y만)
 * - IMU Yaw: rad (오일러 각)
 * - Position: m (x, y)
 * - P 제어 기반 회전 및 이동
 *********************************************************************/
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "/sportmodestate"
#define PI 3.14159265358979323846

// 웨이포인트 구조체
struct Waypoint {
    double x;
    double y;
    std::string name;
};

enum NavigationStage {
    STAGE_TURN_TO_WAYPOINT,
    STAGE_MOVE_TO_WAYPOINT,
    STAGE_ARRIVED,
    STAGE_COMPLETED
};

class WaypointNavigator : public rclcpp::Node {
public:
    WaypointNavigator()
    : Node("waypoint_navigator"),
      stage_(STAGE_TURN_TO_WAYPOINT),
      current_waypoint_idx_(0),
      position_ready_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_rad_(0.0),
      target_yaw_rad_(0.0)
    {
        // CSV 파일 경로 파라미터
        this->declare_parameter<std::string>("csv_file", "waypoints.csv");
        std::string csv_file = this->get_parameter("csv_file").as_string();
        
        // CSV에서 웨이포인트 로드
        if (!loadWaypointsFromCSV(csv_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints!");
            rclcpp::shutdown();
            return;
        }
        
        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded!");
            rclcpp::shutdown();
            return;
        }
        
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            TOPIC_HIGHSTATE, 10,
            std::bind(&WaypointNavigator::stateCallback, this, std::placeholders::_1));
        
        req_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for position data...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WaypointNavigator::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Waypoint Navigator Started ===");
        RCLCPP_INFO(this->get_logger(), "Total waypoints: %zu", waypoints_.size());
        printCurrentWaypoint();
    }

private:
    // CSV 로드
    bool loadWaypointsFromCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open: %s", filename.c_str());
            return false;
        }
        
        std::string line;
        int line_number = 0;
        
        while (std::getline(file, line)) {
            line_number++;
            if (line.empty()) continue;
            
            std::stringstream ss(line);
            std::string x_str, y_str;
            
            if (!std::getline(ss, x_str, ',')) continue;
            if (!std::getline(ss, y_str, ',')) continue;
            
            try {
                Waypoint wp;
                wp.x = std::stod(x_str);
                wp.y = std::stod(y_str);
                wp.name = "WP" + std::to_string(waypoints_.size() + 1);
                waypoints_.push_back(wp);
                
                RCLCPP_INFO(this->get_logger(), "Loaded: %s (%.2f, %.2f)", 
                           wp.name.c_str(), wp.x, wp.y);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Line %d error: %s", line_number, e.what());
            }
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", 
                   waypoints_.size(), filename.c_str());
        return !waypoints_.empty();
    }
    
    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        current_x_ = msg->position[0];
        current_y_ = msg->position[1];
        current_yaw_rad_ = msg->imu_state.rpy[2];
        position_ready_ = true;
    }
    
    double normalizeRad(double ang) {
        while (ang >  PI) ang -= 2.0 * PI;
        while (ang < -PI) ang += 2.0 * PI;
        return ang;
    }
    
    double calculateDistance() {
        Waypoint wp = waypoints_[current_waypoint_idx_];
        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;
        return sqrt(dx*dx + dy*dy);
    }
    
    double calculateTargetYaw() {
        Waypoint wp = waypoints_[current_waypoint_idx_];
        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;
        return atan2(dy, dx);
    }
    
    double getYawError() {
        return normalizeRad(target_yaw_rad_ - current_yaw_rad_);
    }
    
    void printCurrentWaypoint() {
        if (current_waypoint_idx_ >= waypoints_.size()) return;
        Waypoint wp = waypoints_[current_waypoint_idx_];
        RCLCPP_INFO(this->get_logger(), "Target: %s (%.2f, %.2f) | Current: (%.2f, %.2f)",
                    wp.name.c_str(), wp.x, wp.y, current_x_, current_y_);
    }
    
    void controlLoop() {
        if (!position_ready_) return;
        
        switch (stage_) {
            
        case STAGE_TURN_TO_WAYPOINT: {
            target_yaw_rad_ = calculateTargetYaw();
            double yaw_err_rad = getYawError();
            double yaw_err_deg = yaw_err_rad * 180.0 / PI;
            
            const double yaw_tolerance = 0.1 * PI / 180.0;
            if (std::fabs(yaw_err_rad) < yaw_tolerance) {
                RCLCPP_INFO(this->get_logger(), "Turn complete!");
                sendStop();
                stage_ = STAGE_MOVE_TO_WAYPOINT;
                break;
            }
            
            const double Kp_yaw = 20.0;
            double w_cmd = Kp_yaw * yaw_err_rad;
            
            const double min_w = 0.15;
            const double max_w = 1.0;
            if (w_cmd > 0.0 && w_cmd < min_w) w_cmd = min_w;
            if (w_cmd < 0.0 && w_cmd > -min_w) w_cmd = -min_w;
            if (w_cmd >  max_w) w_cmd =  max_w;
            if (w_cmd < -max_w) w_cmd = -max_w;
            
            sendMove(0.0f, 0.0f, (float)w_cmd);
            
            static int turn_log_counter = 0;
            if (++turn_log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "Turning... w=%.2f, cur=%.1f°, tgt=%.1f°, err=%.1f°",
                           w_cmd, current_yaw_rad_*180/PI, 
                           target_yaw_rad_*180/PI, yaw_err_deg);
            }
            break;
        }
            
        case STAGE_MOVE_TO_WAYPOINT: {
            double distance = calculateDistance();
            
            const double arrival_tolerance = 0.15;
            if (distance < arrival_tolerance) {
                RCLCPP_INFO(this->get_logger(), "=== Arrived at %s! ===",
                           waypoints_[current_waypoint_idx_].name.c_str());
                sendStop();
                stage_ = STAGE_ARRIVED;
                break;
            }
            
            const double Kp_distance = 1;
            double speed = Kp_distance * distance;
            
            const double min_speed = 0.3;
            const double max_speed = 1.0;
            if (speed < min_speed) speed = min_speed;
            if (speed > max_speed) speed = max_speed;
            
            double heading_error = std::fabs(getYawError()) * 180.0 / PI;
            const double heading_error_threshold = 15.0;
            
            if (heading_error > heading_error_threshold) {
                RCLCPP_WARN(this->get_logger(),
                           "Heading error %.1f°, re-turning...", heading_error);
                sendStop();
                stage_ = STAGE_TURN_TO_WAYPOINT;
                break;
            }
            
            sendMove((float)speed, 0.0f, 0.0f);
            
            static int move_log_counter = 0;
            if (++move_log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "Moving... speed=%.2f m/s, dist=%.2f m, heading_err=%.1f°",
                           speed, distance, heading_error);
            }
            break;
        }
            
        case STAGE_ARRIVED: {
            current_waypoint_idx_++;
            
            if (current_waypoint_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "🎉 All waypoints completed! 🎉");
                stage_ = STAGE_COMPLETED;
            } else {
                RCLCPP_INFO(this->get_logger(), "--- Moving to next waypoint [%d/%zu] ---",
                           current_waypoint_idx_ + 1, waypoints_.size());
                printCurrentWaypoint();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                stage_ = STAGE_TURN_TO_WAYPOINT;
            }
            break;
        }
            
        case STAGE_COMPLETED: {
            sendStop();
            break;
        }
        }
    }
    
    const int32_t ROBOT_SPORT_API_ID_MOVE     = 1008;
    const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
    
    void sendMove(float vx, float vy, float vyaw) {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
        
        std::string param = "{\"x\":" + std::to_string(vx) +
                           ",\"y\":" + std::to_string(vy) +
                           ",\"z\":" + std::to_string(vyaw) + "}";
        req.parameter = param;
        req_pub_->publish(req);
    }
    
    void sendStop() {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
        req_pub_->publish(req);
    }
    
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_idx_;
    
    NavigationStage stage_;
    bool position_ready_;
    
    double current_x_;
    double current_y_;
    double current_yaw_rad_;
    double target_yaw_rad_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}