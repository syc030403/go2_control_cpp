#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <fstream>
#include <vector>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <cmath>

struct Waypoint {
    double x;
    double y;
    double yaw;
    double timestamp;
};

class WaypointRecorder : public rclcpp::Node
{
public:
    WaypointRecorder() : Node("waypoint_recorder")
    {
        // 파라미터 선언
        this->declare_parameter("recording_interval", 0.5);
        this->declare_parameter("min_distance", 0.5);
        this->declare_parameter("min_rotation_deg", 60.0);  // 새 파라미터: 회전 임계값
        this->declare_parameter("output_file", "waypoints.csv");
        
        recording_interval_ = this->get_parameter("recording_interval").as_double();
        min_distance_ = this->get_parameter("min_distance").as_double();
        min_rotation_deg_ = this->get_parameter("min_rotation_deg").as_double();
        output_file_ = this->get_parameter("output_file").as_string();
        
        min_rotation_rad_ = min_rotation_deg_ * M_PI / 180.0;  // 라디안 변환
        
        // SportModeState 구독
        state_subscriber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate",
            10,
            std::bind(&WaypointRecorder::stateCallback, this, std::placeholders::_1)
        );
        
        // 서비스 생성
        start_recording_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_recording",
            std::bind(&WaypointRecorder::startRecordingCallback, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        stop_recording_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_recording",
            std::bind(&WaypointRecorder::stopRecordingCallback, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        save_waypoints_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_waypoints",
            std::bind(&WaypointRecorder::saveWaypointsCallback, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 타이머 생성
        recording_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(recording_interval_),
            std::bind(&WaypointRecorder::recordingTimerCallback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Recorder Node Started");
        RCLCPP_INFO(this->get_logger(), "Recording interval: %.2f seconds", recording_interval_);
        RCLCPP_INFO(this->get_logger(), "Minimum distance: %.2f meters", min_distance_);
        RCLCPP_INFO(this->get_logger(), "Minimum rotation: %.1f degrees", min_rotation_deg_);
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "\n=== Commands ===");
        RCLCPP_INFO(this->get_logger(), "Start recording: ros2 service call /start_recording std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "Stop recording: ros2 service call /stop_recording std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "Save to CSV: ros2 service call /save_waypoints std_srvs/srv/Trigger");
    }

private:
    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        current_x_ = msg->position[0];
        current_y_ = msg->position[1];
        current_yaw_ = msg->imu_state.rpy[2];
        
        if (!state_received_) {
            state_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First state received. Ready to record.");
        }
    }
    
    // 각도 차이를 -π ~ π 범위로 정규화
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // 두 각도 사이의 최단 각도 차이 계산
    double calculateAngleDifference(double angle1, double angle2)
    {
        double diff = normalizeAngle(angle2 - angle1);
        return std::abs(diff);
    }
    
    void recordingTimerCallback()
    {
        if (!is_recording_ || !state_received_) {
            return;
        }
        
        // 첫 웨이포인트이거나 충분히 이동/회전했을 때만 기록
        if (waypoints_.empty()) {
            addWaypoint();
            RCLCPP_INFO(this->get_logger(), "First waypoint recorded at (%.2f, %.2f, %.2f deg)", 
                       current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
        } else {
            const Waypoint& last_wp = waypoints_.back();
            
            // 거리 계산
            double distance = calculateDistance(
                current_x_, current_y_,
                last_wp.x, last_wp.y
            );
            
            // 회전 각도 계산
            double rotation = calculateAngleDifference(last_wp.yaw, current_yaw_);
            double rotation_deg = rotation * 180.0 / M_PI;
            
            // 거리 또는 회전 조건 만족 시 웨이포인트 추가
            bool distance_condition = distance >= min_distance_;
            bool rotation_condition = rotation >= min_rotation_rad_;
            
            if (distance_condition || rotation_condition) {
                addWaypoint();
                
                std::string reason = "";
                if (distance_condition && rotation_condition) {
                    reason = "(dist: " + std::to_string((int)(distance*100)) + "cm, rot: " + 
                            std::to_string((int)rotation_deg) + "deg)";
                } else if (distance_condition) {
                    reason = "(distance: " + std::to_string((int)(distance*100)) + "cm)";
                } else {
                    reason = "(rotation: " + std::to_string((int)rotation_deg) + "deg)";
                }
                
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu recorded at (%.2f, %.2f, %.2f deg) %s", 
                           waypoints_.size(), current_x_, current_y_, 
                           current_yaw_ * 180.0 / M_PI, reason.c_str());
            }
        }
    }
    
    void addWaypoint()
    {
        Waypoint wp;
        wp.x = current_x_;
        wp.y = current_y_;
        wp.yaw = current_yaw_;
        wp.timestamp = this->now().seconds();
        waypoints_.push_back(wp);
    }
    
    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }
    
    void startRecordingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!state_received_) {
            response->success = false;
            response->message = "Cannot start recording: No state data received yet";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        if (is_recording_) {
            response->success = false;
            response->message = "Already recording";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        is_recording_ = true;
        waypoints_.clear();
        
        response->success = true;
        response->message = "Recording started. Drive the robot to record waypoints.";
        RCLCPP_INFO(this->get_logger(), "\n===========================================");
        RCLCPP_INFO(this->get_logger(), "RECORDING STARTED");
        RCLCPP_INFO(this->get_logger(), "Drive the robot to record waypoints");
        RCLCPP_INFO(this->get_logger(), "Conditions: %.2fm OR %.0f degrees rotation", 
                   min_distance_, min_rotation_deg_);
        RCLCPP_INFO(this->get_logger(), "===========================================");
    }
    
    void stopRecordingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_recording_) {
            response->success = false;
            response->message = "Not currently recording";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        is_recording_ = false;
        
        response->success = true;
        response->message = "Recording stopped. Total waypoints: " + 
                           std::to_string(waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "\n===========================================");
        RCLCPP_INFO(this->get_logger(), "RECORDING STOPPED");
        RCLCPP_INFO(this->get_logger(), "Total waypoints recorded: %zu", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Use /save_waypoints service to save to CSV");
        RCLCPP_INFO(this->get_logger(), "===========================================");
    }
    
    void saveWaypointsCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.empty()) {
            response->success = false;
            response->message = "No waypoints to save";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 타임스탬프가 포함된 파일명 생성
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        
        std::string timestamped_filename = "waypoints_" + oss.str() + ".csv";
        
        std::ofstream file(timestamped_filename);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Failed to open file: " + timestamped_filename;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // CSV 헤더 작성
        file << "x,y,yaw\n";
        
        // 웨이포인트 데이터 작성, 소수점 1자리
        for (const auto& wp : waypoints_) {
            file << std::fixed << std::setprecision(1)
                 << wp.x << ","
                 << wp.y << ","
                 << wp.yaw << "\n";
        }
        file.close();
        
        // 기본 파일명으로도 저장
        std::ofstream default_file(output_file_);
        if (default_file.is_open()) {
            default_file << "x,y,yaw\n";
            for (const auto& wp : waypoints_) {
                default_file << std::fixed << std::setprecision(1)
                            << wp.x << ","
                            << wp.y << ","
                            << wp.yaw << "\n";
            }
            default_file.close();
        }
        
        response->success = true;
        response->message = "Waypoints saved to: " + timestamped_filename + 
                           " and " + output_file_;
        
        RCLCPP_INFO(this->get_logger(), "\n===========================================");
        RCLCPP_INFO(this->get_logger(), "WAYPOINTS SAVED");
        RCLCPP_INFO(this->get_logger(), "File 1: %s", timestamped_filename.c_str());
        RCLCPP_INFO(this->get_logger(), "File 2: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Total waypoints: %zu", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "===========================================");
        
        // 저장된 웨이포인트 요약 출력
        if (waypoints_.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "\nFirst waypoint: (%.2f, %.2f, %.2f deg)",
                       waypoints_.front().x, waypoints_.front().y, 
                       waypoints_.front().yaw * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "Last waypoint: (%.2f, %.2f, %.2f deg)",
                       waypoints_.back().x, waypoints_.back().y, 
                       waypoints_.back().yaw * 180.0 / M_PI);
            
            double total_distance = 0.0;
            for (size_t i = 1; i < waypoints_.size(); i++) {
                total_distance += calculateDistance(
                    waypoints_[i-1].x, waypoints_[i-1].y,
                    waypoints_[i].x, waypoints_[i].y
                );
            }
            RCLCPP_INFO(this->get_logger(), "Total path length: %.2f meters", total_distance);
        }
    }
    
    // Member variables
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_recording_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_waypoints_service_;
    rclcpp::TimerBase::SharedPtr recording_timer_;
    
    std::vector<Waypoint> waypoints_;
    
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    
    bool is_recording_ = false;
    bool state_received_ = false;
    
    double recording_interval_;
    double min_distance_;
    double min_rotation_deg_;  // 도 단위
    double min_rotation_rad_;  // 라디안 단위
    std::string output_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}