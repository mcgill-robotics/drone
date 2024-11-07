#include <connection_result.h>
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>

#include "custom_msgs/msg/local_pos_vel.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_controller") {

    // tf2 stuff
    tf_world_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_world_frame();

    tf_drone_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // MAVSDK stuff
    mavsdk = std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{
        mavsdk::Mavsdk::ComponentType::GroundStation});
    mavsdk::ConnectionResult con_res =
        mavsdk->add_any_connection("udp://:14540");

    if (con_res != mavsdk::ConnectionResult::Success) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Cannot connect to drone, maybe modify the connection address");
      std::exit(1);
    }

    sys = mavsdk->first_autopilot(3.0);
    if (!sys) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for system");
      std::exit(2);
    }

    mavsdk_telem = std::make_unique<mavsdk::Telemetry>(sys.value());
    this->setup_monitoring();

    // ROS stuff
    monitoring_topic = this->create_publisher<custom_msgs::msg::LocalPosVel>(
        "drone_position", 10);
    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&OffboardNode::monitoring_callback, this));
  }

private:
  void make_world_frame() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "None";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    t.transform.rotation.x = 1;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 0;

    this->tf_world_broadcaster->sendTransform(t);
  }

  void setup_monitoring() {

    msg.header.frame_id = "world";
    transform.header.frame_id = "world";
    transform.child_frame_id = "drone";

    mavsdk_telem->subscribe_position_velocity_ned(
        [this](mavsdk::Telemetry::PositionVelocityNed pos) {
          msg.n = pos.position.north_m;
          msg.e = pos.position.east_m;
          msg.d = pos.position.down_m;

          msg.vn = pos.velocity.north_m_s;
          msg.ve = pos.velocity.east_m_s;
          msg.vd = pos.velocity.down_m_s;

          transform.transform.translation.x = pos.position.north_m;
          transform.transform.translation.y = pos.position.east_m;
          transform.transform.translation.z = pos.position.down_m;
        });

    auto gps_orig = this->mavsdk_telem->get_gps_global_origin();
    msg.origin_lon = gps_orig.second.longitude_deg;
    msg.origin_lat = gps_orig.second.latitude_deg;
    msg.origin_alt = gps_orig.second.altitude_m;

    mavsdk_telem->subscribe_attitude_quaternion(
        [this](mavsdk::Telemetry::Quaternion quat) {
          msg.x = quat.x;
          msg.y = quat.y;
          msg.z = quat.z;
          msg.w = quat.w;
          transform.transform.rotation.x = quat.x;
          transform.transform.rotation.y = quat.y;
          transform.transform.rotation.z = quat.z;
          transform.transform.rotation.w = quat.w;
        });

    mavsdk_telem->subscribe_attitude_angular_velocity_body(
        [this](mavsdk::Telemetry::AngularVelocityBody angular_vel) {
          msg.roll_rad_s = angular_vel.roll_rad_s;
          msg.pitch_rad_s = angular_vel.pitch_rad_s;
          msg.yaw_rad_s = angular_vel.yaw_rad_s;
        });
  }

  void monitoring_callback() {
    msg.header.stamp = this->get_clock()->now();
    transform.header.stamp = this->get_clock()->now();
    this->tf_drone_broadcaster->sendTransform(transform);
    this->monitoring_topic->publish(msg);
  }

  // ROS2 related stuff
  rclcpp::Publisher<custom_msgs::msg::LocalPosVel>::SharedPtr monitoring_topic;
  custom_msgs::msg::LocalPosVel msg;
  rclcpp::TimerBase::SharedPtr timer_;

  // tf2 - transforms for Rviz visualization
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_world_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_drone_broadcaster;
  geometry_msgs::msg::TransformStamped transform;

  // mavsdk related stuff
  std::optional<std::shared_ptr<mavsdk::System>> sys;
  std::unique_ptr<mavsdk::Mavsdk> mavsdk;
  std::unique_ptr<mavsdk::Telemetry> mavsdk_telem;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
