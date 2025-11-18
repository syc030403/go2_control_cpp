/**********************************************************************
 * GO2 1초 전진 + 0.5초 정지 + 정확히 90도 회전 + 0.5초 정지 반복
 * - ROS2 Foxy
 * - IMU Yaw: rad (오일러 각)
 * - 명령 z: rad/s
 * - 단순 P제어 기반 회전 (라디안 통일)
 *********************************************************************/
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "/sportmodestate"
#define PI 3.14159265358979323846

enum WalkStage {
    STAGE_MOVE_FORWARD,
    STAGE_STOP_1,
    STAGE_TURN_LEFT,
    STAGE_STOP_2
};

class SimpleGo2Walker : public rclcpp::Node {
public:
    SimpleGo2Walker()
    : Node("simple_go2_walker"),
      stage_(STAGE_MOVE_FORWARD),
      stage_time_(0.0),
      yaw_ready_(false),
      current_yaw_rad_(0.0),
      target_yaw_rad_(0.0)
    {
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            TOPIC_HIGHSTATE, 10,
            std::bind(&SimpleGo2Walker::stateCallback, this, std::placeholders::_1));

        req_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        RCLCPP_INFO(this->get_logger(), "Waiting for IMU...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SimpleGo2Walker::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Simple Square Walker (rad ver.) Started");
    }

private:
    // IMU 콜백: yaw(rad) 그대로 사용
    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        // imu_state.rpy[2] 가 라디안이라고 확정
        current_yaw_rad_ = msg->imu_state.rpy[2];
        yaw_ready_ = true;
    }

    // 라디안 -PI ~ PI 범위로 정규화
    double normalizeRad(double ang) {
        while (ang >  PI) ang -= 2.0 * PI;
        while (ang < -PI) ang += 2.0 * PI;
        return ang;
    }

    void controlLoop() {
        if (!yaw_ready_) return;      // IMU 안 들어오면 대기

        stage_time_ += 0.01;          // 타이머 10ms 기준

        switch (stage_) {
        case STAGE_MOVE_FORWARD:
            // 1초 동안 전진
            sendMove(0.3f, 0.0f, 0.0f);

            if (stage_time_ < 0.05) {
                RCLCPP_INFO(this->get_logger(), "Forward...");
            }

            if (stage_time_ > 1.0) {
                stage_ = STAGE_STOP_1;
                stage_time_ = 0.0;
                sendStop();
            }
            break;

        case STAGE_STOP_1:
            // 0.5초 정지 후, 현재 yaw 기준 +90도(= PI/2 rad) 타겟 설정
            if (stage_time_ < 0.05) {
                RCLCPP_INFO(this->get_logger(), "Stop after forward");
            }

            if (stage_time_ > 0.5) {
                target_yaw_rad_ = normalizeRad(current_yaw_rad_ + PI / 2.0);
                RCLCPP_INFO(this->get_logger(),
                            "Start turn: cur = %.1f deg, tgt = %.1f deg",
                            current_yaw_rad_ * 180.0 / PI,
                            target_yaw_rad_ * 180.0 / PI);
                stage_ = STAGE_TURN_LEFT;
                stage_time_ = 0.0;
            }
            break;

        case STAGE_TURN_LEFT: { // 방향 값을 줄 때 +, -로 주므로 나중에 이름을 STAGE_TURN 으로 바꿀것
            // 각도 오차 (rad)
            double yaw_err_rad = normalizeRad(target_yaw_rad_ - current_yaw_rad_);

            // 허용 오차: 0.1도 ≒ 0.1*pi/180 rad
            const double tol_rad = 0.1 * PI / 180.0;

            if (std::fabs(yaw_err_rad) < tol_rad) {
                RCLCPP_INFO(this->get_logger(),
                            "Turn done. err = %.2f deg",
                            yaw_err_rad * 180.0 / PI);
                sendStop();
                stage_ = STAGE_STOP_2;
                stage_time_ = 0.0;
                break;
            }

            // ----- P 제어 (rad → rad/s) -----
            // Kp: [1/s] (각도(rad) * Kp = 각속도(rad/s))
            const double Kp = 20.0;          // 여기 게인만 튜닝하면 됨
            //게인: 약간 가중치 느낌, 게인에 err값(각)이 계속 곱해지는데 결국 err은 0에 수렴하므로 90도에 도달하면 멈추게 됨
            double w_cmd = Kp * yaw_err_rad;

            // 최소 / 최대 회전 속도
            const double min_w = 0.15;      // rad/s
            const double max_w = 1.0;       // rad/s

            // 최소 속도 보장
            if (w_cmd > 0.0 && w_cmd < min_w)  w_cmd = min_w;
            if (w_cmd < 0.0 && w_cmd > -min_w) w_cmd = -min_w;
            // 최대 속도 제한
            if (w_cmd >  max_w) w_cmd =  max_w;
            if (w_cmd < -max_w) w_cmd = -max_w;
            // --------------------------------

            sendMove(0.0f, 0.0f, (float)w_cmd);

            // 로그 조금만
            if (((int)(stage_time_ * 100)) % 20 == 0) {
                RCLCPP_INFO(this->get_logger(),
                            "Turning... w=%.2f rad/s, cur=%.1f deg, tgt=%.1f deg, err=%.1f deg",
                            w_cmd,
                            current_yaw_rad_ * 180.0 / PI,
                            target_yaw_rad_ * 180.0 / PI,
                            yaw_err_rad * 180.0 / PI);
            }
            break;
        }

        case STAGE_STOP_2:
            // 회전 이후 0.5초 정지
            if (stage_time_ < 0.05) {
                RCLCPP_INFO(this->get_logger(), "Stop after turn");
            }

            if (stage_time_ > 0.5) {
                stage_ = STAGE_MOVE_FORWARD;
                stage_time_ = 0.0;
            }
            break;
        }
    }

    // ==== GO2 명령 부분 ====
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

    // ROS 멤버
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr          req_pub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;

    // 상태
    WalkStage stage_;
    double    stage_time_;

    // IMU yaw (rad)
    bool   yaw_ready_;
    double current_yaw_rad_;
    double target_yaw_rad_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleGo2Walker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}