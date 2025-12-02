/**********************************************************************
 * GO2 Waypoint Navigator MK3 - Improved Version
 * - ROS2 Foxy
 * - CSV 파일에서 웨이포인트 로드 (x,y만)
 * - 회전 + 직진 동시 제어 (부드러운 곡선 주행)
 * - 향상된 PD 제어 (빠른 수렴, 높은 속도)
 * - 후진 회피 동작
 * 
 * 개선 사항:
 * 1. 속도 향상: 더 높은 Kp_speed, max_speed
 * 2. 각도 수렴 개선: 향상된 Kp_yaw, anti-windup, 적응형 게인
 * 3. 회피시 후진: escape시 vx를 음수로 변경
 * 4. 부드러운 회전: heading_factor 개선, 최소 속도 보장
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
    STAGE_STUCK,         // 정체 감지 - 후진 회피 동작
    STAGE_ARRIVED,
    STAGE_COMPLETED
};

class WaypointNavigatorMK3 : public rclcpp::Node {
public:
    WaypointNavigatorMK3()
    : Node("waypoint_navigator_mk3"),
      stage_(STAGE_NAVIGATE),
      current_waypoint_idx_(0),
      position_ready_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_rad_(0.0),
      // PD 제어용
      prev_yaw_error_(0.0),
      prev_distance_(0.0),
      yaw_integral_(0.0),
      prev_time_(this->now()),
      // 정체 감지용
      prev_position_x_(0.0),
      prev_position_y_(0.0),
      stuck_start_time_(this->now()),
      escape_start_time_(this->now()),
      position_check_time_(this->now()),
      is_stuck_(false)
    {
        // CSV 파일 경로 파라미터
        this->declare_parameter<std::string>("csv_file", "waypoints.csv");
        std::string csv_file = this->get_parameter("csv_file").as_string();
        
        // ============================================
        // 개선된 PD 게인 (더 빠른 응답)
        // ============================================
        this->declare_parameter<double>("Kp_yaw", 3.5);      // 2.0 -> 3.5 (더 빠른 회전)
        this->declare_parameter<double>("Kd_yaw", 0.15);     // 0.3 -> 0.15 (덜 감쇠)
        this->declare_parameter<double>("Ki_yaw", 0.1);      // 새로 추가 (정상상태 오차 제거)
        this->declare_parameter<double>("Kp_speed", 1.5);    // 0.8 -> 1.5 (더 빠른 직진)
        this->declare_parameter<double>("Kd_speed", 0.05);   // 0.1 -> 0.05
        
        // 속도 제한 파라미터
        this->declare_parameter<double>("max_linear_speed", 1.0);   // 최대 전진 속도
        this->declare_parameter<double>("min_linear_speed", 0.5);  // 최소 전진 속도 (정지 방지)
        this->declare_parameter<double>("max_angular_speed", 1.57);  // 최대 회전 속도
        
        // 정체 감지 파라미터
        this->declare_parameter<double>("stuck_timeout", 3.0);
        this->declare_parameter<double>("stuck_distance_threshold", 0.05);  // 3초간 이동 거리 임계값
        this->declare_parameter<double>("escape_duration", 1.5);    // 후진 시간
        this->declare_parameter<double>("escape_speed", 0.4);       // 후진 속도
        
        // 도착 판정
        this->declare_parameter<double>("arrival_tolerance", 0.15);
        
        // 게인 로드
        Kp_yaw_ = this->get_parameter("Kp_yaw").as_double();
        Kd_yaw_ = this->get_parameter("Kd_yaw").as_double();
        Ki_yaw_ = this->get_parameter("Ki_yaw").as_double();
        Kp_speed_ = this->get_parameter("Kp_speed").as_double();
        Kd_speed_ = this->get_parameter("Kd_speed").as_double();
        
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        min_linear_speed_ = this->get_parameter("min_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        
        stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
        stuck_distance_threshold_ = this->get_parameter("stuck_distance_threshold").as_double();
        escape_duration_ = this->get_parameter("escape_duration").as_double();
        escape_speed_ = this->get_parameter("escape_speed").as_double();
        arrival_tolerance_ = this->get_parameter("arrival_tolerance").as_double();
        
        RCLCPP_INFO(this->get_logger(), "=== MK3 Configuration ===");
        RCLCPP_INFO(this->get_logger(), "PID Gains - Yaw: Kp=%.2f, Kd=%.2f, Ki=%.2f",
                   Kp_yaw_, Kd_yaw_, Ki_yaw_);
        RCLCPP_INFO(this->get_logger(), "PD Gains - Speed: Kp=%.2f, Kd=%.2f",
                   Kp_speed_, Kd_speed_);
        RCLCPP_INFO(this->get_logger(), "Speed Limits - Linear: %.2f~%.2f m/s, Angular: %.2f rad/s",
                   min_linear_speed_, max_linear_speed_, max_angular_speed_);
        RCLCPP_INFO(this->get_logger(), "Stuck Detection - Timeout: %.1fs, Threshold: %.2fm",
                   stuck_timeout_, stuck_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "Escape - Duration: %.1fs, Speed: %.2f m/s (BACKWARD)",
                   escape_duration_, escape_speed_);
        
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
            std::bind(&WaypointNavigatorMK3::stateCallback, this, std::placeholders::_1));
        
        req_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for position data...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 제어 루프 주기: 20ms (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&WaypointNavigatorMK3::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Waypoint Navigator MK3 Started ===");
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
        if (dt <= 0.0) dt = 0.02;  // 최소값 보장
        prev_time_ = current_time;
        
        switch (stage_) {
            
        case STAGE_NAVIGATE: {
            double distance = calculateDistance();
            double yaw_error = getYawError();
            double yaw_error_deg = yaw_error * 180.0 / PI;
            
            // ============================================
            // 도착 판정
            // ============================================
            if (distance < arrival_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "=== Arrived at %s! (dist=%.3fm) ===",
                           waypoints_[current_waypoint_idx_].name.c_str(), distance);
                sendStop();
                stage_ = STAGE_ARRIVED;
                resetControlState();
                break;
            }
            
            // ============================================
            // 정체 감지 (위치 기반)
            // ============================================
            double time_since_check = (current_time - position_check_time_).seconds();
            
            if (time_since_check >= 1.0) {  // 1초마다 체크
                double moved_distance = sqrt(
                    pow(current_x_ - prev_position_x_, 2) + 
                    pow(current_y_ - prev_position_y_, 2)
                );
                
                if (moved_distance < stuck_distance_threshold_) {
                    if (!is_stuck_) {
                        stuck_start_time_ = current_time;
                        is_stuck_ = true;
                        RCLCPP_WARN(this->get_logger(), 
                                   "[STUCK] Possible stuck detected, monitoring... (moved %.3fm in 1s)",
                                   moved_distance);
                    }
                    
                    double stuck_duration = (current_time - stuck_start_time_).seconds();
                    
                    if (stuck_duration >= stuck_timeout_) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "[STUCK] Confirmed! No progress for %.1fs. Starting BACKWARD escape.",
                                   stuck_duration);
                        
                        stage_ = STAGE_STUCK;
                        escape_start_time_ = current_time;
                        is_stuck_ = false;
                        break;
                    }
                } else {
                    is_stuck_ = false;
                }
                
                // 위치 업데이트
                prev_position_x_ = current_x_;
                prev_position_y_ = current_y_;
                position_check_time_ = current_time;
            }
            
            // ============================================
            // PID 제어: 회전 (Yaw) - 빠른 수렴
            // ============================================
            double yaw_error_derivative = (yaw_error - prev_yaw_error_) / dt;
            
            // Integral term with anti-windup
            yaw_integral_ += yaw_error * dt;
            const double integral_limit = 1.0;
            yaw_integral_ = std::clamp(yaw_integral_, -integral_limit, integral_limit);
            
            // 적응형 게인: 큰 오차일 때 더 강하게 반응
            double adaptive_Kp = Kp_yaw_;
            if (std::abs(yaw_error_deg) > 45.0) {
                adaptive_Kp = Kp_yaw_ * 1.5;  // 45도 이상이면 게인 증가
            }
            
            double w_cmd = adaptive_Kp * yaw_error 
                         + Kd_yaw_ * yaw_error_derivative 
                         + Ki_yaw_ * yaw_integral_;
            
            // 회전 속도 제한
            w_cmd = std::clamp(w_cmd, -max_angular_speed_, max_angular_speed_);
            
            // ============================================
            // PD 제어: 직진 (Speed) - 부드러운 곡선 주행
            // ============================================
            double distance_error_derivative = (prev_distance_ - distance) / dt;
            
            // 기본 속도 계산
            double speed_cmd = Kp_speed_ * distance + Kd_speed_ * (-distance_error_derivative);
            
            // ============================================
            // 부드러운 속도 조절 (헤딩 기반)
            // ============================================
            // cos 대신 부드러운 감소 함수 사용
            // 90도 이내: 속도 유지, 90도 이상: 점진적 감소
            double abs_yaw_error = std::abs(yaw_error);
            double heading_factor;
            
            if (abs_yaw_error < PI / 4) {
                // 45도 이내: 거의 최대 속도
                heading_factor = 1.0;
            } else if (abs_yaw_error < PI / 2) {
                // 45~90도: 선형 감소 (1.0 -> 0.5)
                heading_factor = 1.0 - 0.5 * (abs_yaw_error - PI/4) / (PI/4);
            } else if (abs_yaw_error < 3 * PI / 4) {
                // 90~135도: 최소 속도 유지
                heading_factor = 0.5;
            } else {
                // 135도 이상: 후진이 필요할 수 있으나 일단 최소 속도
                heading_factor = 0.3;
            }
            
            speed_cmd *= heading_factor;
            
            // 최소 속도 보장 (정지 방지)
            // 단, 목표에 가까우면 최소 속도 완화
            double effective_min_speed = min_linear_speed_;
            if (distance < 0.3) {
                effective_min_speed = min_linear_speed_ * 0.5;
            }
            
            // 속도 제한 (최소~최대)
            if (speed_cmd > 0) {
                speed_cmd = std::clamp(speed_cmd, effective_min_speed, max_linear_speed_);
            } else {
                speed_cmd = 0.0;  // 음수 방지
            }
            
            // 이전 오차 저장
            prev_yaw_error_ = yaw_error;
            prev_distance_ = distance;
            
            // ============================================
            // 명령 전송 (vx, vy, vyaw 동시 - 부드러운 곡선)
            // ============================================
            sendMove((float)speed_cmd, 0.0f, (float)w_cmd);
            
            // 로그 출력 (500ms마다, 50Hz * 25 = 500ms)
            static int log_counter = 0;
            if (++log_counter % 25 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "[NAV] dist=%.2fm, yaw_err=%.1f deg | vx=%.2f, vyaw=%.2f | hf=%.2f",
                           distance, yaw_error_deg, speed_cmd, w_cmd, heading_factor);
            }
            break;
        }
        
        case STAGE_STUCK: {
            // ============================================
            // 회피 동작: 후진 (BACKWARD)
            // ============================================
            double escape_elapsed = (this->now() - escape_start_time_).seconds();
            
            if (escape_elapsed < escape_duration_) {
                // 후진 + 약간의 회전 (장애물에서 벗어나기)
                double escape_vyaw = 0.3;  // 약간 회전하면서 후진
                sendMove(-(float)escape_speed_, 0.0f, (float)escape_vyaw);
                
                static int escape_log_counter = 0;
                if (++escape_log_counter % 25 == 0) {
                    RCLCPP_WARN(this->get_logger(), 
                               "[ESCAPE] Moving BACKWARD... (%.1fs / %.1fs)",
                               escape_elapsed, escape_duration_);
                }
            } else {
                // 회피 완료 - 웨이포인트 스킵
                sendStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                
                Waypoint skipped_wp = waypoints_[current_waypoint_idx_];
                RCLCPP_WARN(this->get_logger(), 
                           "[SKIP] Waypoint %s (%.2f, %.2f) skipped due to stuck",
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
                    
                    resetControlState();
                    
                    // 위치 체크 리셋
                    prev_position_x_ = current_x_;
                    prev_position_y_ = current_y_;
                    position_check_time_ = this->now();
                    
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
                
                // 위치 체크 리셋
                prev_position_x_ = current_x_;
                prev_position_y_ = current_y_;
                position_check_time_ = this->now();
                
                // ::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    
    void resetControlState() {
        prev_yaw_error_ = 0.0;
        prev_distance_ = 0.0;
        yaw_integral_ = 0.0;
        is_stuck_ = false;
    }
    
    // ============================================
    // Robot API
    // ============================================
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
    
    // ============================================
    // ROS2 Members
    // ============================================
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
    
    // ============================================
    // PID Control Gains (Improved)
    // ============================================
    double Kp_yaw_;
    double Kd_yaw_;
    double Ki_yaw_;
    double Kp_speed_;
    double Kd_speed_;
    
    // Speed Limits
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    
    // PID Control - State
    double prev_yaw_error_;
    double prev_distance_;
    double yaw_integral_;
    rclcpp::Time prev_time_;
    
    // ============================================
    // Stuck Detection (Position-based)
    // ============================================
    double stuck_timeout_;
    double stuck_distance_threshold_;
    double escape_duration_;
    double escape_speed_;
    double arrival_tolerance_;
    
    double prev_position_x_;
    double prev_position_y_;
    rclcpp::Time stuck_start_time_;
    rclcpp::Time escape_start_time_;
    rclcpp::Time position_check_time_;
    bool is_stuck_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigatorMK3>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}