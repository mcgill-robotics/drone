// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <connection_result.h>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <memory>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <string>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_controller") {
    // ROS stuff
    monitoring_topic = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "drone_position", 10);

    // MAVSDK stuff
    monitoring_topic = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "drone_position", 10);
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
    const auto set_rate_res = mavsdk_telem->set_rate_position(0.5);
    if (set_rate_res != mavsdk::Telemetry::Result::Success) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't set rate for telemetry %d",
                   set_rate_res);
      std::exit(3);
    }

    mavsdk_telem->subscribe_position([this](mavsdk::Telemetry::Position pos) {
      auto msg = geometry_msgs::msg::PointStamped();
      msg.point.x = pos.latitude_deg;
      msg.point.y = pos.longitude_deg;
      msg.point.z = pos.relative_altitude_m;
      this->monitoring_topic->publish(msg);
    });

    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //     "topic", 10, std::bind(&OffboardNode::topic_callback, this,
    //     std::placeholders::_1));
    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&OffboardNode::monitoring_callback, this));
  }

private:
  // void monitoring_callback() {
  //   auto msg = geometry_msgs::msg::PointStamped();
  //   msg.point.x = 10.;
  //   msg.point.y = 20.;
  //   msg.point.z = 30.;
  //   monitoring_topic->publish(msg);
  // }

  // void topic_callback(const std_msgs::msg::String &msg) const {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  // }

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // ROS2 related stuff
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      monitoring_topic;
  rclcpp::TimerBase::SharedPtr timer_;

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
