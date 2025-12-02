/**********************************************************************
 * GO2 Waypoint Navigator with PD Control + Stuck Detection
 * - ROS2 Foxy
 * - CSV 파일에서 웨이포인트 로드 (x,y만)
 * - 회전 + 직진 동시 제어
 * - PD 제어 기반
 * - 정체 감지 및 회피 기능 (장애물 회피 + 웨이포인트 스킵)
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
    STAGE_NAVIGATE,      // 회전 + 직진 동시 수행
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
      // 정체 감지용
      prev_imu_vx_(0.0),
      stuck_start_time_(this->now()),
      escape_start_time_(this->now()),
      is_stuck_(false)
    {
        // CSV 파일 경로 파라미터
        this->declare_parameter<std::string>("csv_file", "waypoints.csv");
        std::string csv_file = this->get_parameter("csv_file").as_string();
        
        // PD 게인 파라미터 (튜닝 가능)
        this->declare_parameter<double>("Kp_yaw", 2.0);
        this->declare_parameter<double>("Kd_yaw", 0.3);
        this->declare_parameter<double>("Kp_speed", 0.8);
        this->declare_parameter<double>("Kd_speed", 0.1);
        
        // 정체 감지 파라미터
        this->declare_parameter<double>("stuck_timeout", 3.0);
        this->declare_parameter<double>("angle_tolerance", 10.0);
        this->declare_parameter<double>("speed_change_threshold", 0.05);
        this->declare_parameter<double>("escape_duration", 2.0);
        this->declare_parameter<double>("escape_speed", 0.5);
        
        Kp_yaw_ = this->get_parameter("Kp_yaw").as_double();
        Kd_yaw_ = this->get_parameter("Kd_yaw").as_double();
        Kp_speed_ = this->get_parameter("Kp_speed").as_double();
        Kd_speed_ = this->get_parameter("Kd_speed").as_double();
        
        stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        speed_change_threshold_ = this->get_parameter("speed_change_threshold").as_double();
        escape_duration_ = this->get_parameter("escape_duration").as_double();
        escape_speed_ = this->get_parameter("escape_speed").as_double();
        
        RCLCPP_INFO(this->get_logger(), "PD Gains - Yaw: Kp=%.2f, Kd=%.2f | Speed: Kp=%.2f, Kd=%.2f",
                   Kp_yaw_, Kd_yaw_, Kp_speed_, Kd_speed_);
        RCLCPP_INFO(this->get_logger(), "Stuck Detection - Timeout: %.1fs, Angle: %.1fdeg, Speed: %.2fm/s",
                   stuck_timeout_, angle_tolerance_, speed_change_threshold_);
        
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
        
        RCLCPP_INFO(this->get_logger(), "=== Waypoint Navigator PD (with Stuck Detection) Started ===");
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
        
        // IMU 속도 저장 (정체 감지용)
        current_imu_vx_ = msg->velocity[0];
        
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
        if (dt <= 0.0) dt = 0.01;  // 최소값 보장
        prev_time_ = current_time;
        
        switch (stage_) {
            
        case STAGE_NAVIGATE: {
            double distance = calculateDistance();
            double yaw_error = getYawError();
            double yaw_error_deg = yaw_error * 180.0 / PI;
            
            // 도착 판정
            const double arrival_tolerance = 0.15;
            if (distance < arrival_tolerance) {
                RCLCPP_INFO(this->get_logger(), "=== Arrived at %s! ===",
                           waypoints_[current_waypoint_idx_].name.c_str());
                sendStop();
                stage_ = STAGE_ARRIVED;
                // 다음 웨이포인트를 위해 초기화
                prev_yaw_error_ = 0.0;
                prev_distance_ = 0.0;
                is_stuck_ = false;
                break;
            }
            
            // ============================================
            // 정체 감지 (Stuck Detection)
            // ============================================
            // 1. 각도 수렴 체크
            bool angle_converged = (std::abs(yaw_error_deg) < angle_tolerance_);
            
            // 2. IMU 속도 변화 체크
            double imu_speed_change = std::abs(current_imu_vx_ - prev_imu_vx_);
            bool speed_stagnant = (imu_speed_change < speed_change_threshold_);
            
            // 3. 정체 상태 판정
            if (!angle_converged || speed_stagnant) {
                // 정체 시작 시간 기록
                if (!is_stuck_) {
                    stuck_start_time_ = this->now();
                    is_stuck_ = true;
                }
                
                double stuck_duration = (this->now() - stuck_start_time_).seconds();
                
                // 3초 이상 정체 시 회피 모드로 전환
                if (stuck_duration > stuck_timeout_) {
                    RCLCPP_WARN(this->get_logger(), 
                               "[STUCK] Detected! Angle converged: %s, Speed stagnant: %s (%.1fs)",
                               angle_converged ? "YES" : "NO",
                               speed_stagnant ? "YES" : "NO",
                               stuck_duration);
                    RCLCPP_WARN(this->get_logger(), 
                               "Attempting escape maneuver...");
                    
                    stage_ = STAGE_STUCK;
                    escape_start_time_ = this->now();
                    is_stuck_ = false;
                    break;
                }
            } else {
                // 정상 상태 복귀
                is_stuck_ = false;
            }
            
            // IMU 속도 업데이트
            prev_imu_vx_ = current_imu_vx_;
            
            // ============================================
            // PD 제어: 회전 (Yaw)
            // ============================================
            double yaw_error_derivative = (yaw_error - prev_yaw_error_) / dt;
            double w_cmd = Kp_yaw_ * yaw_error + Kd_yaw_ * yaw_error_derivative;
            
            // 회전 속도 제한
            const double max_w = 1.5;
            w_cmd = std::clamp(w_cmd, -max_w, max_w);
            
            // ============================================
            // PD 제어: 직진 (Speed)
            // ============================================
            double distance_derivative = (prev_distance_ - distance) / dt;  // 음수면 접근 중
            
            // 거리 오차에 비례 + 접근 속도(음의 미분)에 따른 감쇠
            double speed_cmd = Kp_speed_ * distance + Kd_speed_ * (-distance_derivative);
            
            // 헤딩 오차가 클 때 속도 감소 (곡선 주행 방지)
            double heading_factor = std::cos(yaw_error);  // -1 ~ 1
            if (heading_factor < 0.0) heading_factor = 0.0;  // 뒤를 향하면 전진 안함
            
            speed_cmd *= heading_factor;
            
            // 속도 제한
            const double min_speed = 0.0;
            const double max_speed = 1.0;
            speed_cmd = std::clamp(speed_cmd, min_speed, max_speed);
            
            // 이전 오차 저장
            prev_yaw_error_ = yaw_error;
            prev_distance_ = distance;
            
            // 명령 전송 (vx, vy, vyaw 동시)
            sendMove((float)speed_cmd, 0.0f, (float)w_cmd);
            
            // 로그 출력 (500ms마다)
            static int log_counter = 0;
            if (++log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "[PD] dist=%.2fm, yaw_err=%.1f deg | vx=%.2f, w=%.2f | heading_factor=%.2f",
                           distance, yaw_error_deg, speed_cmd, w_cmd, heading_factor);
            }
            break;
        }
        
        case STAGE_STUCK: {
            // 회피 동작: 일정 시간 동안 전진
            double escape_elapsed = (this->now() - escape_start_time_).seconds();
            
            if (escape_elapsed < escape_duration_) {
                // 전진 중
                sendMove((float)escape_speed_, 0.0f, 0.0f);
                
                static int escape_log_counter = 0;
                if (++escape_log_counter % 50 == 0) {
                    RCLCPP_WARN(this->get_logger(), 
                               "[ESCAPE] Moving forward... (%.1fs / %.1fs)",
                               escape_elapsed, escape_duration_);
                }
            } else {
                // 회피 완료 - 웨이포인트 스킵
                sendStop();
                
                Waypoint skipped_wp = waypoints_[current_waypoint_idx_];
                RCLCPP_WARN(this->get_logger(), 
                           "[SKIP] Waypoint %s (%.2f, %.2f) skipped due to stuck condition",
                           skipped_wp.name.c_str(), skipped_wp.x, skipped_wp.y);
                
                current_waypoint_idx_++;
                
                if (current_waypoint_idx_ >= waypoints_.size()) {
                    RCLCPP_INFO(this->get_logger(), "=== All waypoints completed! ===");
                    stage_ = STAGE_COMPLETED;
                } else {
                    RCLCPP_INFO(this->get_logger(), 
                               "--- Moving to next waypoint [%zu/%zu] after skip ---",
                               current_waypoint_idx_ + 1, waypoints_.size());
                    printCurrentWaypoint();
                    
                    // 상태 초기화
                    prev_yaw_error_ = 0.0;
                    prev_distance_ = 0.0;
                    is_stuck_ = false;
                    
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
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                stage_ = STAGE_NAVIGATE;
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
    
    // Stuck Detection Parameters
    double stuck_timeout_;           // 정체 판정 시간 (초)
    double angle_tolerance_;         // 각도 수렴 허용 오차 (도)
    double speed_change_threshold_;  // 속도 변화 최소값 (m/s)
    double escape_duration_;         // 회피 전진 시간 (초)
    double escape_speed_;            // 회피 전진 속도 (m/s)
    
    // Stuck Detection State
    double prev_imu_vx_;             // 이전 IMU 속도 (x축)
    double current_imu_vx_;          // 현재 IMU 속도 (x축)
    rclcpp::Time stuck_start_time_;  // 정체 시작 시간
    rclcpp::Time escape_start_time_; // 회피 시작 시간
    bool is_stuck_;                  // 정체 상태 플래그
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigatorPD>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}