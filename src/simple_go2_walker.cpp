/**********************************************************************
 * GO2 Waypoint Navigator
 * - ROS2 Foxy
 * - IMU Yaw: rad (오일러 각)
 * - Position: m (x, y)
 * - P 제어 기반 회전 및 이동
 *********************************************************************/
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "/sportmodestate"
#define PI 3.14159265358979323846

// 웨이포인트 구조체
struct Waypoint {
    double x;
    double y;
    std::string name;  // 디버깅용
};

enum NavigationStage {
    STAGE_TURN_TO_WAYPOINT,    // 웨이포인트 방향으로 회전
    STAGE_MOVE_TO_WAYPOINT,    // 웨이포인트로 이동
    STAGE_ARRIVED,             // 웨이포인트 도착
    STAGE_COMPLETED            // 모든 웨이포인트 완료
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
        // 웨이포인트 정의 (정사각형 경로)
        waypoints_ = {
            {1.0, 0.0, "WP1"},   // 1m 전진
            {1.0, 1.0, "WP2"},   // 좌회전 후 1m
            {0.0, 1.0, "WP3"},   // 좌회전 후 1m
            {0.0, 0.0, "WP4"}    // 좌회전 후 1m (시작점)
        };
        
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
    // ========== 콜백 및 유틸리티 함수 ==========
    
    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        // 위치 및 자세 업데이트
        current_x_ = msg->position[0];
        current_y_ = msg->position[1];
        current_yaw_rad_ = msg->imu_state.rpy[2];
        position_ready_ = true;
    }
    
    // 라디안 정규화 (-PI ~ PI)
    double normalizeRad(double ang) {
        while (ang >  PI) ang -= 2.0 * PI;
        while (ang < -PI) ang += 2.0 * PI;
        return ang;
    }
    
    // ========== 네비게이션 계산 함수 ==========
    
    // 현재 웨이포인트까지의 거리
    double calculateDistance() {
        Waypoint wp = waypoints_[current_waypoint_idx_];
        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;
        return sqrt(dx*dx + dy*dy);
    }
    
    // 현재 웨이포인트를 향한 목표 각도
    double calculateTargetYaw() {
        Waypoint wp = waypoints_[current_waypoint_idx_];
        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;
        return atan2(dy, dx);  // -PI ~ PI
    }
    
    // 현재 각도와 목표 각도의 오차
    double getYawError() {
        return normalizeRad(target_yaw_rad_ - current_yaw_rad_);
    }
    
    // 웨이포인트 정보 출력
    void printCurrentWaypoint() {
        if (current_waypoint_idx_ >= waypoints_.size()) return;
        Waypoint wp = waypoints_[current_waypoint_idx_];
        RCLCPP_INFO(this->get_logger(), 
                    "Target: %s (%.2f, %.2f) | Current: (%.2f, %.2f)",
                    wp.name.c_str(), wp.x, wp.y, current_x_, current_y_);
    }
    
    // ========== 메인 제어 루프 ==========
    
    void controlLoop() {
        if (!position_ready_) return;  // 위치 데이터 대기
        
        switch (stage_) {
            
        case STAGE_TURN_TO_WAYPOINT: {
            // 웨이포인트 방향으로 회전
            target_yaw_rad_ = calculateTargetYaw();
            double yaw_err_rad = getYawError();
            double yaw_err_deg = yaw_err_rad * 180.0 / PI;
            
            // 회전 완료 판정
            const double yaw_tolerance = 3.0 * PI / 180.0;  // 3도
            if (std::fabs(yaw_err_rad) < yaw_tolerance) {
                RCLCPP_INFO(this->get_logger(), 
                           "Turn complete! Starting movement...");
                sendStop();
                stage_ = STAGE_MOVE_TO_WAYPOINT;
                break;
            }
            
            // P 제어 (회전)
            const double Kp_yaw = 20.0;
            double w_cmd = Kp_yaw * yaw_err_rad;
            
            // 속도 제한
            const double min_w = 0.15;
            const double max_w = 1.0;
            if (w_cmd > 0.0 && w_cmd < min_w) w_cmd = min_w;
            if (w_cmd < 0.0 && w_cmd > -min_w) w_cmd = -min_w;
            if (w_cmd >  max_w) w_cmd =  max_w;
            if (w_cmd < -max_w) w_cmd = -max_w;
            
            sendMove(0.0f, 0.0f, (float)w_cmd);
            
            // 주기적 로그
            static int turn_log_counter = 0;
            if (++turn_log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "Turning... w=%.2f rad/s, cur=%.1f°, tgt=%.1f°, err=%.1f°",
                           w_cmd, current_yaw_rad_*180/PI, 
                           target_yaw_rad_*180/PI, yaw_err_deg);
            }
            break;
        }
            
        case STAGE_MOVE_TO_WAYPOINT: {
            // 웨이포인트로 이동
            double distance = calculateDistance();
            
            // 도착 판정
            const double arrival_tolerance = 0.15;  // 15cm
            if (distance < arrival_tolerance) {
                RCLCPP_INFO(this->get_logger(),
                           "=== Arrived at %s! Distance: %.3f m ===",
                           waypoints_[current_waypoint_idx_].name.c_str(),
                           distance);
                sendStop();
                stage_ = STAGE_ARRIVED;
                break;
            }
            
            // P 제어 (거리 기반 속도)
            const double Kp_distance = 0.5;
            double speed = Kp_distance * distance;
            
            // 속도 제한
            const double min_speed = 0.1;   // m/s
            const double max_speed = 0.4;   // m/s
            if (speed < min_speed) speed = min_speed;
            if (speed > max_speed) speed = max_speed;
            
            // 방향 오차 확인 (이동 중 방향 틀어지면 다시 회전)
            double heading_error = std::fabs(getYawError()) * 180.0 / PI;
            const double heading_error_threshold = 15.0;  // 15도
            
            if (heading_error > heading_error_threshold) {
                RCLCPP_WARN(this->get_logger(),
                           "Heading error too large (%.1f°), re-turning...",
                           heading_error);
                sendStop();
                stage_ = STAGE_TURN_TO_WAYPOINT;
                break;
            }
            
            sendMove((float)speed, 0.0f, 0.0f);
            
            // 주기적 로그
            static int move_log_counter = 0;
            if (++move_log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                           "Moving... speed=%.2f m/s, distance=%.2f m, heading_err=%.1f°",
                           speed, distance, heading_error);
            }
            break;
        }
            
        case STAGE_ARRIVED: {
            // 다음 웨이포인트로
            current_waypoint_idx_++;
            
            if (current_waypoint_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), 
                           "🎉 All waypoints completed! 🎉");
                stage_ = STAGE_COMPLETED;
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "--- Moving to next waypoint [%d/%zu] ---",
                           current_waypoint_idx_ + 1, waypoints_.size());
                printCurrentWaypoint();
                
                // 0.5초 대기 후 다음 웨이포인트로
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                stage_ = STAGE_TURN_TO_WAYPOINT;
            }
            break;
        }
            
        case STAGE_COMPLETED: {
            // 완료 상태 유지 (또는 재시작)
            sendStop();
            
            // 선택: 다시 처음부터 시작
            // current_waypoint_idx_ = 0;
            // stage_ = STAGE_TURN_TO_WAYPOINT;
            break;
        }
        }
    }
    
    // ========== GO2 명령 함수 ==========
    
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
    
    // ========== 멤버 변수 ==========
    
    // ROS
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 웨이포인트
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_idx_;
    
    // 상태
    NavigationStage stage_;
    bool position_ready_;
    
    // 로봇 위치/자세
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