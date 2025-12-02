/**********************************************************************
 * GO2 Waypoint Navigator with Smooth Curve Driving
 * - ROS2 Foxy
 * - CSV 파일에서 웨이포인트 로드 (x,y만)
 * - vx와 vyaw 동시 제어로 부드러운 곡선 주행
 * - PD 제어 기반
 * - 속도 2.0
 * - 거리 변화 기반 정체 감지 및 회피 기능
 * - 딜레이 발생 시키는 sendstop 제거
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

struct Waypoint {
    double x;
    double y;
    std::string name;
};

enum NavigationStage {
    STAGE_NAVIGATE,      // 곡선 주행으로 웨이포인트 접근
    STAGE_STUCK,         // 정체 감지 - 회피 동작
    STAGE_ARRIVED,
    STAGE_COMPLETED
};

class WaypointNavigatorPD : public rclcpp::Node {
public:
    WaypointNavigatorPD()
    : Node("waypoint_navigator_pd"),
      stage_(STAGE_NAVIGATE),
      current_waypoint_idx_(0),
      position_ready_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_rad_(0.0),
      // PD 제어용
      prev_yaw_error_(0.0),
      prev_distance_(0.0),
      prev_time_(this->now()),
      // 거리 기반 정체 감지용
      prev_check_distance_(0.0),
      distance_check_time_(this->now()),
      stuck_start_time_(this->now()),
      escape_start_time_(this->now()),
      is_stuck_(false)
    {
        // CSV 파일 경로 파라미터
        this->declare_parameter<std::string>("csv_file", "waypoints.csv");
        std::string csv_file = this->get_parameter("csv_file").as_string();
        
        // PD 게인 파라미터 (튜닝 가능)
        this->declare_parameter<double>("Kp_yaw", 2.5);
        this->declare_parameter<double>("Kd_yaw", 0.4);
        this->declare_parameter<double>("Kp_speed", 0.6);
        this->declare_parameter<double>("Kd_speed", 0.15);
        
        // 부드러운 주행을 위한 파라미터
        this->declare_parameter<double>("base_speed", 0.5);           // 기본 주행 속도
        this->declare_parameter<double>("min_curve_speed", 0.2);      // 곡선 주행 시 최소 속도
        this->declare_parameter<double>("curve_slowdown_angle", 45.0); // 이 각도 이상이면 감속 시작 (도)
        
        // 거리 기반 정체 감지 파라미터
        this->declare_parameter<double>("stuck_timeout", 3.0);
        this->declare_parameter<double>("angle_tolerance", 10.0);
        this->declare_parameter<double>("distance_check_interval", 0.5);
        this->declare_parameter<double>("distance_change_threshold", 0.05);
        this->declare_parameter<double>("escape_duration", 2.0);
        this->declare_parameter<double>("escape_speed", -0.3);  // 음수 = 후진
        
        Kp_yaw_ = this->get_parameter("Kp_yaw").as_double();
        Kd_yaw_ = this->get_parameter("Kd_yaw").as_double();
        Kp_speed_ = this->get_parameter("Kp_speed").as_double();
        Kd_speed_ = this->get_parameter("Kd_speed").as_double();
        
        base_speed_ = this->get_parameter("base_speed").as_double();
        min_curve_speed_ = this->get_parameter("min_curve_speed").as_double();
        curve_slowdown_angle_ = this->get_parameter("curve_slowdown_angle").as_double();
        
        stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        distance_check_interval_ = this->get_parameter("distance_check_interval").as_double();
        distance_change_threshold_ = this->get_parameter("distance_change_threshold").as_double();
        escape_duration_ = this->get_parameter("escape_duration").as_double();
        escape_speed_ = this->get_parameter("escape_speed").as_double();
        
        RCLCPP_INFO(this->get_logger(), "=== Smooth Curve Driving Configuration ===");
        RCLCPP_INFO(this->get_logger(), "PD Gains - Yaw: Kp=%.2f, Kd=%.2f | Speed: Kp=%.2f, Kd=%.2f",
                   Kp_yaw_, Kd_yaw_, Kp_speed_, Kd_speed_);
        RCLCPP_INFO(this->get_logger(), "Curve Driving - Base: %.2fm/s, Min: %.2fm/s, Slowdown Angle: %.1fdeg",
                   base_speed_, min_curve_speed_, curve_slowdown_angle_);
        RCLCPP_INFO(this->get_logger(), "Stuck Detection - Timeout: %.1fs, Distance Change: %.2fm/%.1fs",
                   stuck_timeout_, distance_change_threshold_, distance_check_interval_);
        
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
            std::bind(&WaypointNavigatorPD::stateCallback, this, std::placeholders::_1));
        
        req_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for position data...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WaypointNavigatorPD::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Waypoint Navigator (Smooth Curve Mode) Started ===");
        RCLCPP_INFO(this->get_logger(), "Total waypoints: %zu", waypoints_.size());
        printCurrentWaypoint();
    }

private:
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
        current_x_ = std::round(msg->position[0] * 10.0) / 10.0;
        current_y_ = std::round(msg->position[1] * 10.0) / 10.0;
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
        double target_yaw = calculateTargetYaw();
        return normalizeRad(target_yaw - current_yaw_rad_);
    }
    
    void printCurrentWaypoint() {
        if (current_waypoint_idx_ >= waypoints_.size()) return;
        Waypoint wp = waypoints_[current_waypoint_idx_];
        RCLCPP_INFO(this->get_logger(), "Target: %s (%.2f, %.2f) | Current: (%.2f, %.2f)",
                    wp.name.c_str(), wp.x, wp.y, current_x_, current_y_);
    }
    
    void controlLoop() {
        if (!position_ready_) return;
        
        // dt 계산
        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 0.0) dt = 0.01;
        prev_time_ = current_time;
        
        switch (stage_) {
            
        case STAGE_NAVIGATE: {
            double distance = calculateDistance();
            double yaw_error = getYawError();
            double yaw_error_deg = yaw_error * 180.0 / PI;
            double abs_yaw_error_deg = std::abs(yaw_error_deg);
            
            // 도착 판정
            if (distance < 0.1) {
                RCLCPP_INFO(this->get_logger(), "[ARRIVED] Waypoint reached! Distance: %.2fm", distance);
                
                stage_ = STAGE_ARRIVED;
                break;
            }
            
            // ============================================
            // 거리 변화 기반 정체 감지
            // ============================================
            double time_since_check = (current_time - distance_check_time_).seconds();
            
            if (time_since_check >= distance_check_interval_) {
                double distance_change = std::abs(prev_check_distance_ - distance);
                bool angle_converged = abs_yaw_error_deg < angle_tolerance_;
                bool distance_stagnant = (distance_change < distance_change_threshold_);
                
                if (angle_converged || distance_stagnant) {
                    if (!is_stuck_) {
                        stuck_start_time_ = this->now();
                        is_stuck_ = true;
                        RCLCPP_WARN(this->get_logger(), 
                                   "[STUCK?] Monitoring... (angle: %s, distance: %s)",
                                   angle_converged ? "converged" : "NOT converged",
                                   distance_stagnant ? "stagnant" : "NOT stagnant");
                    }
                    
                    double stuck_duration = (this->now() - stuck_start_time_).seconds();
                    
                    if (stuck_duration > stuck_timeout_) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "[STUCK] Distance change: %.3fm in %.1fs (threshold: %.3fm)",
                                   distance_change, distance_check_interval_, distance_change_threshold_);
                        RCLCPP_WARN(this->get_logger(), "Attempting escape maneuver...");
                        
                        stage_ = STAGE_STUCK;
                        escape_start_time_ = this->now();
                        is_stuck_ = false;
                        break;
                    }
                } else {
                    if (is_stuck_) {
                        RCLCPP_INFO(this->get_logger(), "[RECOVERED] Normal movement detected.");
                    }
                    is_stuck_ = false;
                }
                
                prev_check_distance_ = distance;
                distance_check_time_ = current_time;
            }
            
            // ============================================
            // PD 제어: 회전 (Yaw) - 부드러운 곡선을 위한 회전 속도
            // ============================================
            double yaw_error_derivative = (yaw_error - prev_yaw_error_) / dt;
            double w_cmd = Kp_yaw_ * yaw_error + Kd_yaw_ * yaw_error_derivative;
            
            // 회전 속도 제한
            const double max_w = 1.5;
            w_cmd = std::clamp(w_cmd, -max_w, max_w);
            
            // ============================================
            // 부드러운 곡선 주행을 위한 전진 속도 제어
            // ============================================
            
            // 방법 1: 거리 기반 속도 (목표에 가까우면 감속)
            double distance_derivative = (prev_distance_ - distance) / dt;
            double speed_from_distance = Kp_speed_ * distance + Kd_speed_ * (-distance_derivative);
            
            // 방법 2: 각도 기반 속도 감속 (곡선이 급하면 감속)
            double speed_from_angle;
            if (abs_yaw_error_deg > curve_slowdown_angle_) {
                // 급한 곡선: 각도 오차에 따라 속도 감소
                // 90도 오차 -> min_curve_speed, 0도 오차 -> base_speed
                double angle_factor = 1.0 - (abs_yaw_error_deg / 90.0);  // 0 ~ 1
                angle_factor = std::clamp(angle_factor, 0.0, 1.0);
                speed_from_angle = min_curve_speed_ + (base_speed_ - min_curve_speed_) * angle_factor;
            } else {
                // 완만한 곡선: 기본 속도 유지
                speed_from_angle = base_speed_;
            }
            
            // 두 방법 중 작은 값 선택 (안전하게)
            double speed_cmd = std::min(speed_from_distance, speed_from_angle);
            
            // 뒤를 향하면 속도 0 (180도 근처)
            if (std::abs(yaw_error_deg) > 90.0) {
                speed_cmd = 0.0;
            }
            
            // 속도 제한
            const double min_speed = 1.0;
            const double max_speed = 2.0;
            speed_cmd = std::clamp(speed_cmd, min_speed, max_speed);
            
            // 이전 오차 저장
            prev_yaw_error_ = yaw_error;
            prev_distance_ = distance;
            
            // ============================================
            // vx와 vyaw 동시 전송 -> 부드러운 곡선 주행
            // ============================================
            sendMove((float)speed_cmd, 0.0f, (float)w_cmd);
            
            // 로그 출력 (500ms마다)
            static int log_counter = 0;
            if (++log_counter % 50 == 0) {
                std::string curve_mode;
                if (abs_yaw_error_deg > curve_slowdown_angle_) {
                    curve_mode = "SHARP CURVE";
                } else if (abs_yaw_error_deg > angle_tolerance_) {
                    curve_mode = "GENTLE CURVE";
                } else {
                    curve_mode = "STRAIGHT";
                }
                
                RCLCPP_INFO(this->get_logger(),
                           "[%s] dist=%.2fm, yaw=%.1fdeg | vx=%.2f, w=%.2f | stuck=%s",
                           curve_mode.c_str(), distance, yaw_error_deg, 
                           speed_cmd, w_cmd, is_stuck_ ? "YES" : "NO");
            }
            break;
        }
        
        case STAGE_STUCK: {
            double escape_elapsed = (this->now() - escape_start_time_).seconds();
            
            if (escape_elapsed < escape_duration_) {
                sendMove((float)escape_speed_, 0.0f, 0.0f);
                
                static int escape_log_counter = 0;
                if (++escape_log_counter % 50 == 0) {
                    RCLCPP_WARN(this->get_logger(), 
                               "[ESCAPE] Moving backward... (%.1fs / %.1fs)",
                               escape_elapsed, escape_duration_);
                }
            } else {
                sendStop();
                
                Waypoint skipped_wp = waypoints_[current_waypoint_idx_];
                RCLCPP_WARN(this->get_logger(), 
                           "[SKIP] Waypoint %s (%.2f, %.2f) skipped",
                           skipped_wp.name.c_str(), skipped_wp.x, skipped_wp.y);
                
                current_waypoint_idx_++;
                
                if (current_waypoint_idx_ >= waypoints_.size()) {
                    RCLCPP_INFO(this->get_logger(), "=== All waypoints completed! ===");
                    stage_ = STAGE_COMPLETED;
                } else {
                    RCLCPP_INFO(this->get_logger(), 
                               "--- Moving to next waypoint [%zu/%zu] ---",
                               current_waypoint_idx_ + 1, waypoints_.size());
                    printCurrentWaypoint();
                    
                    prev_yaw_error_ = 0.0;
                    prev_distance_ = 0.0;
                    is_stuck_ = false;
                    prev_check_distance_ = calculateDistance();
                    distance_check_time_ = this->now();
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    stage_ = STAGE_NAVIGATE;
                }
            }
            break;
        }
            
        case STAGE_ARRIVED: {
            current_waypoint_idx_++;
            
            if (current_waypoint_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "=== All waypoints completed! ===");
                stage_ = STAGE_COMPLETED;
            } else {
                RCLCPP_INFO(this->get_logger(), "--- Moving to next waypoint [%zu/%zu] ---",
                           current_waypoint_idx_ + 1, waypoints_.size());
                printCurrentWaypoint();
                
                // PD 제어 이전 값 초기화
                prev_yaw_error_ = 0.0;
                prev_distance_ = calculateDistance();
                
                // 거리 기반 정체 감지 초기화
                prev_check_distance_ = prev_distance_;
                distance_check_time_ = this->now();
                is_stuck_ = false;
                
                // 대기 없이 바로 다음 웨이포인트로 이동
                stage_ = STAGE_NAVIGATE;
            }
            break;
        }
            
        case STAGE_COMPLETED: {
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
    
    // ROS2
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Waypoints
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_idx_;
    
    // State
    NavigationStage stage_;
    bool position_ready_;
    
    // Position/Orientation
    double current_x_;
    double current_y_;
    double current_yaw_rad_;
    
    // PD Control Gains
    double Kp_yaw_;
    double Kd_yaw_;
    double Kp_speed_;
    double Kd_speed_;
    
    // PD Control - Previous values
    double prev_yaw_error_;
    double prev_distance_;
    rclcpp::Time prev_time_;
    
    // Smooth Curve Driving Parameters
    double base_speed_;              // 기본 주행 속도
    double min_curve_speed_;         // 곡선 주행 시 최소 속도
    double curve_slowdown_angle_;    // 감속 시작 각도 (도)
    
    // Distance-based Stuck Detection Parameters
    double stuck_timeout_;
    double angle_tolerance_;
    double distance_check_interval_;
    double distance_change_threshold_;
    double escape_duration_;
    double escape_speed_;
    
    // Distance-based Stuck Detection State
    double prev_check_distance_;
    rclcpp::Time distance_check_time_;
    rclcpp::Time stuck_start_time_;
    rclcpp::Time escape_start_time_;
    bool is_stuck_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigatorPD>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}