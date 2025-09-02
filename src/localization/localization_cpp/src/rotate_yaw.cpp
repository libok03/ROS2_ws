#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <deque>
#include <cmath>

// ★ 너희 패키지 메시지 헤더 (이름/경로는 프로젝트에 맞게 조정)
#include <erp42_msgs/msg/serial_feed_back.hpp>
using erp42_msgs::msg::SerialFeedBack;

class Rotate : public rclcpp::Node
{
public:
  Rotate()
  : Node("rotate_yaw"),
    delta_yaw_(this->declare_parameter<double>("delta_yaw", 0.0)),
    freeze_speed_thresh_(this->declare_parameter<double>("freeze_speed_thresh", 0.05)), // m/s
    freeze_zero_gyro_accel_(this->declare_parameter<bool>("freeze_zero_gyro_accel", true)),
    fb_topic_(this->declare_parameter<std::string>("erp42_feedback_topic", "erp42_feedback"))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", qos, std::bind(&Rotate::imuCallback, this, std::placeholders::_1));

    sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "ublox_gps_node/fix_velocity", qos, std::bind(&Rotate::gpsVelCallback, this, std::placeholders::_1));

    sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", qos, std::bind(&Rotate::gpsFixCallback, this, std::placeholders::_1));

    sub_gps_jam_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps_jaming", qos, std::bind(&Rotate::gpsJammingCallback, this, std::placeholders::_1));

    // ★ ERP42 피드백 구독: estop, speed 사용
    sub_erp42_fb_ = create_subscription<SerialFeedBack>(
      fb_topic_, qos, std::bind(&Rotate::erp42FeedbackCallback, this, std::placeholders::_1));

    pub_imu_rotated_ = create_publisher<sensor_msgs::msg::Imu>("imu/rotated", qos);
    pub_mean_quat_   = create_publisher<geometry_msgs::msg::Quaternion>("mean", qos);
  }

private:
  // --------- 보조 함수 ---------
  static void eulerFromQuat(const tf2::Quaternion &q, double &r, double &p, double &y)
  { tf2::Matrix3x3(q).getRPY(r, p, y); }

  static tf2::Quaternion quatFromEuler(double r, double p, double y)
  { tf2::Quaternion q; q.setRPY(r, p, y); return q; }

  bool decisionStraight()
  {
    if (forward_.size() < 10) return false;
    double sum = 0.0; for (double a : forward_) sum += a;
    double mean = sum / forward_.size();
    for (double a : forward_)
      if (std::abs((mean - a) * 180.0 / M_PI) > 2.0) return false;
    mean_ = mean; return true;
  }

  // --------- 콜백 ---------
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  { cov_ = msg->position_covariance[0]; }

  void gpsJammingCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  { jamming_cov0_ = msg->pose.covariance[0]; }

  void gpsVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    v_gps_ = std::hypot(vx, vy);
    gps_yaw_ = std::atan2(vy, vx);

    forward_.push_back(gps_yaw_);
    if (forward_.size() > 10) forward_.pop_front();
  }

  // ★ ERP42 feedback: estop, speed 반영
  void erp42FeedbackCallback(const SerialFeedBack::SharedPtr msg)
  {
    // 보내준 로그 기준: estop: 0/1, speed: m/s 로 가정
    estop_fb_ = (msg->estop != 0);
    v_erp42_  = msg->speed;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // (1) 현재 yaw
    double roll, pitch, yaw;
    tf2::Quaternion q_orig(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    eulerFromQuat(q_orig, roll, pitch, yaw);

    // (2) GPS-IMU 차이 → delta_ 갱신 (직진+신뢰 조건)
    const double delta   = gps_yaw_ - yaw;
    const double abs_deg = std::abs(delta) * 180.0 / M_PI;

    if (abs_deg > 2.0 && abs_deg < 90.0 &&
        v_gps_ > 0.50 && cov_ < 0.0004 && jamming_cov0_ < 0.0004)
    {
      if (decisionStraight()) {
        delta_ = mean_ - yaw;
        tf2::Quaternion q_gps = quatFromEuler(0,0,gps_yaw_);
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q_gps.x(); q_msg.y = q_gps.y(); q_msg.z = q_gps.z(); q_msg.w = q_gps.w();
        pub_mean_quat_->publish(q_msg);
      }
    }

    // (3) FREEZE 조건: ERP42 estop==1 || ERP42 속도 < 임계값
    const bool freeze = estop_fb_ || (v_erp42_ < freeze_speed_thresh_);
    sensor_msgs::msg::Imu imu_out = *msg;

    if (freeze) {
      if (!last_quat_initialized_) { last_quat_pub_ = q_orig; last_quat_initialized_ = true; }

      // orientation 고정
      imu_out.orientation.x = last_quat_pub_.x();
      imu_out.orientation.y = last_quat_pub_.y();
      imu_out.orientation.z = last_quat_pub_.z();
      imu_out.orientation.w = last_quat_pub_.w();

      // (옵션) 각속도/가속도 0 고정
      if (freeze_zero_gyro_accel_) {
        imu_out.angular_velocity.x = 0.0;
        imu_out.angular_velocity.y = 0.0;
        imu_out.angular_velocity.z = 0.0;
        imu_out.linear_acceleration.x = 0.0;
        imu_out.linear_acceleration.y = 0.0;
        imu_out.linear_acceleration.z = 0.0;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[FREEZE] estop=%d, v_erp42=%.3f m/s → orientation held",
        estop_fb_ ? 1 : 0, v_erp42_);
    } else {
      // (4) 보정된 yaw 적용
      const double yaw_prev = yaw;
      yaw = yaw + delta_yaw_ + delta_;
      tf2::Quaternion q_new = quatFromEuler(0,0,yaw);

      imu_out.orientation.x = q_new.x();
      imu_out.orientation.y = q_new.y();
      imu_out.orientation.z = q_new.z();
      imu_out.orientation.w = q_new.w();

      last_quat_pub_ = q_new;
      last_quat_initialized_ = true;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
        "Yaw prev: %.2f°, new: %.2f°, delta: %.2f°",
        yaw_prev * 180.0 / M_PI, yaw * 180.0 / M_PI, delta_ * 180.0 / M_PI);
    }

    // (유지) orientation 공분산
    imu_out.orientation_covariance = {
      0.00025, 0.0,     0.0,
      0.0,     0.00025, 0.0,
      0.0,     0.0,     0.00025
    };

    pub_imu_rotated_->publish(imu_out);
  }

  // --------- 멤버 ---------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_gps_vel_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_jam_;
  rclcpp::Subscription<SerialFeedBack>::SharedPtr sub_erp42_fb_;           // ★ 추가

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_rotated_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_mean_quat_;

  // 직선 판정 / GPS-IMU 정렬
  std::deque<double> forward_;
  double mean_{0.0};
  double delta_{0.0};

  // 파라미터
  double      delta_yaw_;
  double      freeze_speed_thresh_;
  bool        freeze_zero_gyro_accel_;
  std::string fb_topic_;

  // 신호 상태
  double gps_yaw_{0.0};
  double v_gps_{0.0};
  double cov_{0.0};
  double jamming_cov0_{0.0};

  // ERP42 상태
  bool   estop_fb_{false};
  double v_erp42_{0.0};

  // freeze용 마지막 자세
  tf2::Quaternion last_quat_pub_{0,0,0,1};
  bool last_quat_initialized_{false};
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rotate>());
  rclcpp::shutdown();
  return 0;
}
