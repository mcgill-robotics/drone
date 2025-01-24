#include <cmath>
#include <custom_msgs/msg/detail/action__struct.hpp>
#include <custom_msgs/msg/detail/drone_state__struct.hpp>
#include <deque>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/empty__struct.hpp>

#include "custom_msgs/msg/action.hpp"
#include "custom_msgs/msg/drone_state.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/offboard/offboard.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

template <typename... Args>
std::string string_format(const std::string &format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
               1; // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(),
                     buf.get() + size - 1); // We don't want the '\0' inside
}

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_controller"), qc_tolerance(1.) {

    // tf2 stuff
    tf_world_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_world_frame();

    tf_drone_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // MAVSDK stuff
    mavsdk = std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{
        mavsdk::Mavsdk::ComponentType::GroundStation});

    // TODO: Modify the connection url when connected to the PixHawk
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
    mavsdk_action = std::make_unique<mavsdk::Action>(sys.value());
    mavsdk_offboard = std::make_unique<mavsdk::Offboard>(sys.value());
    // const mavsdk::Offboard::VelocityNedYaw stay{};
    mavsdk::Offboard::PositionNedYaw stay_pos{};
    mavsdk_offboard->set_position_ned(stay_pos);

    this->setup_monitoring();

    // ROS stuff
    monitoring_pub =
        this->create_publisher<custom_msgs::msg::DroneState>("drone_state", 10);

    enqueue_sub = this->create_subscription<custom_msgs::msg::Action>(
        "enqueue_command", 10,
        std::bind(&OffboardNode::enqueue_topic_callback, this,
                  std::placeholders::_1));

    pop_sub = this->create_subscription<std_msgs::msg::Empty>(
        "cancel_command", 10,
        std::bind(&OffboardNode::cancel_command_topic_callback, this,
                  std::placeholders::_1));

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&OffboardNode::monitoring_callback, this));

    this->command_timer_ = this->create_wall_timer(
        100ms, std::bind(&OffboardNode::command_queue_callback, this));
  }

private:
  // This only makes sense in the context of the action queue
  struct Command {
    // both perform tick and cancel_command return true if they are done, false
    // otherwise
    virtual bool perform_tick(OffboardNode &on) = 0;
    virtual bool cancel_command(OffboardNode &on) {
      RCLCPP_INFO(on.get_logger(), "This command is not cancelable");
      return perform_tick(on);
    }
    virtual std::string get_description() = 0;

    virtual ~Command() {}
  };

  struct TakeOff : Command {
    TakeOff(double height) : target_height(height) {}

    virtual bool perform_tick(OffboardNode &node) override {
      if (node.state.in_air &&
          abs(node.state.d - (-this->target_height)) <= 1.) {
        node.mavsdk_action->hold();
        // Done, return true
        return true;
      }
      if (!node.state.armed) {
        node.mavsdk_action->arm();
      } else if (node.state.flight_mode !=
                 custom_msgs::msg::DroneState::TAKEOFF) {
        node.mavsdk_action->set_takeoff_altitude(target_height + 1.);
        node.mavsdk_action->takeoff();
      }

      return false;
    }

    virtual std::string get_description() override {
      return string_format("Taking off: target height %lf", target_height);
    }

    ~TakeOff() {}

  private:
    double target_height;
  };

  struct Land : Command {

    Land() {}
    virtual bool perform_tick(OffboardNode &node) override {
      if (node.state.flight_mode != custom_msgs::msg::DroneState::LAND) {
        node.mavsdk_action->land();
      }
      if (!node.state.armed) {
        node.mavsdk_action->hold();
        return true;
      }
      return false;
    }

    virtual std::string get_description() override {
      return string_format("Landing");
    }

    ~Land() {}
  };

  struct GoToQC : Command {
    GoToQC(double n, double e, double d)
        : target_n(n), target_e(e), target_d(d) {}

    virtual bool perform_tick(OffboardNode &node) override {
      double dist_from_target = sqrt(pow((node.state.n - target_n), 2) +
                                     pow((node.state.e - target_e), 2) +
                                     pow((node.state.d - target_d), 2));

      if (dist_from_target >= node.qc_tolerance && !goto_command_sent) {
        mavsdk::Offboard::PositionNedYaw pos;
        pos.north_m = target_n;
        pos.east_m = target_e;
        pos.down_m = target_d;
        pos.yaw_deg = NAN;

        auto result = node.mavsdk_offboard->set_position_ned(pos);
        if (result == mavsdk::Offboard::Result::Success)
          goto_command_sent = true;
      }
      if (node.state.flight_mode != custom_msgs::msg::DroneState::OFFBOARD) {
        node.mavsdk_offboard->start();
        return false;
      }

      return dist_from_target < node.qc_tolerance;
    }

    virtual std::string get_description() override {
      return string_format("Going to %lf, %lf, %lf", target_n, target_e,
                           target_d);
    }

    virtual bool cancel_command(OffboardNode &node) override { return true; }

    ~GoToQC() {}
    double target_n, target_e, target_d;
    bool goto_command_sent = false;
  };

  // TODO: Do this
  struct GoToFW : Command {};
  struct Hold : Command {};

  // struct GoTo : Command {
  //   GoTo(double n, double e, double d, double vn, double ve, double vd,
  //        FlightMode fm) {
  //
  //     target_n = n;
  //     target_e = e;
  //     target_d = d;
  //
  //     target_vn = vn;
  //     target_ve = ve;
  //     target_vd = vd;
  //
  //     desired_mode = fm;
  //   }
  //
  //   virtual bool is_goto() override { return true; }
  //
  //   virtual std::optional<GoToTuple> get_goto_tuple() override {
  //     return std::optional<GoToTuple>(GoToTuple(
  //         target_n, target_e, target_d, target_vn, target_ve, target_vd));
  //   };
  //
  //   virtual std::optional<FlightMode> get_desired_fm() override {
  //     return std::optional<FlightMode>(desired_mode);
  //   }
  //
  //   // TODO: Finish implementing velocity and yaw control
  //   virtual bool perform_tick(OffboardNode &node) override {
  //     if (!start_pos_recorded) {
  //       start_pos_recorded = true;
  //       start_n = node.state.n;
  //       start_e = node.state.e;
  //       start_d = node.state.d;
  //     }
  //
  //     // switch to offboard if not already
  //     if (node.state.flight_mode != custom_msgs::msg::DroneState::OFFBOARD) {
  //       node.mavsdk_offboard->start();
  //     }
  //
  //     // send transition command if need to and if not sent already
  //     if (node.state.vtol_state != desired_mode && !transtion_order_sent) {
  //       mavsdk::Action::Result res;
  //       if (desired_mode == FlightMode::QC) {
  //         res = node.mavsdk_action->transition_to_multicopter();
  //       } else {
  //         res = node.mavsdk_action->transition_to_fixedwing();
  //       }
  //       if (res == mavsdk::Action::Result::Success)
  //         transtion_order_sent = true;
  //       return false;
  //     }
  //
  //     double dist_from_target = sqrt(pow((node.state.n - target_n), 2) +
  //                                    pow((node.state.e - target_e), 2) +
  //                                    pow((node.state.d - target_d), 2));
  //
  //     // if above tolerance and command to go somewhere wasnt sent already
  //     if (((desired_mode == FlightMode::QC && dist_from_target >= 1.) ||
  //          (desired_mode == FlightMode::FW && dist_from_target >= 10.)) &&
  //         !goto_order_sent) {
  //       mavsdk::Offboard::PositionNedYaw pos;
  //       pos.north_m = target_n;
  //       pos.east_m = target_e;
  //       pos.down_m = target_d;
  //
  //       pos.yaw_deg = NAN;
  //       // If there is a next goto action
  //       if (node.command_deque.size() >= 1 &&
  //           node.command_deque[0]->is_goto()) {
  //
  //         // if next goto action is in FW and we're in QC: setup yaw and
  //         // actually over shoot the point so you dont slow down before
  //         trying to transition if (desired_mode == FlightMode::QC &&
  //             node.command_deque[0]->get_desired_fm().value() ==
  //                 FlightMode::FW) {
  //
  //           double overshoot_scale = 1.1;
  //           pos.north_m = start_n + (target_n - start_n) * overshoot_scale;
  //           pos.east_m = start_e + (target_e - start_e) * overshoot_scale;
  //
  //           double next_n, next_e, d_n, d_e;
  //           GoToTuple next_goto =
  //               node.command_deque[0]->get_goto_tuple().value();
  //           next_n = next_goto.n;
  //           next_e = next_goto.e;
  //
  //           d_n = next_n - target_n;
  //           d_e = next_e - target_e;
  //
  //           double angle = atan2(d_e, d_n) * 180. / M_PI;
  //           // atan2 gives -180 to 180 while we want 0 to 360 for yaw control
  //           if (angle < 0)
  //             angle = 360. + angle;
  //
  //           pos.yaw_deg = angle;
  //           RCLCPP_INFO(node.get_logger(),
  //                       "next point is in FW, we're in QC, it is at (%f,%f) "
  //                       "from current target, yaw needed is %f",
  //                       d_n, d_e, pos.yaw_deg);
  //         }
  //       }
  //
  //       auto res = node.mavsdk_offboard->set_position_ned(pos);
  //       if (res == mavsdk::Offboard::Result::Success)
  //         goto_order_sent = true;
  //     }
  //
  //     // if within tolerance and in the desired mode, you're done !
  //     if ((((desired_mode == FlightMode::QC && dist_from_target < 1.) ||
  //           (desired_mode == FlightMode::FW && dist_from_target < 10.))) &&
  //         node.state.vtol_state == desired_mode) {
  //       // if (node.command_deque.size() == ) {
  //       //   node.mavsdk_offboard->stop();
  //       // }
  //       return true;
  //     }
  //     return false;
  //   }
  //
  //   ~GoTo() {}
  //
  //   double start_n, start_e, start_d;
  //   double target_n, target_e, target_d, target_vn, target_ve, target_vd;
  //   FlightMode desired_mode;
  //
  //   // toggles to not overwhelm the flight controller
  //   bool start_pos_recorded = false;
  //   bool transtion_order_sent = false;
  //   bool goto_order_sent = false;
  // };

  // TODO: DO REST OF COMMAND CLASSES

  void make_world_frame() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "none";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    this->tf_world_broadcaster->sendTransform(t);
  }

  void setup_monitoring() {

    state.header.frame_id = "world";
    transform.header.frame_id = "world";
    transform.child_frame_id = "drone";

    mavsdk_telem->subscribe_in_air(
        [this](bool in_air) { state.in_air = in_air; });

    mavsdk_telem->subscribe_armed([this](bool armed) { state.armed = armed; });

    mavsdk_telem->subscribe_vtol_state(
        [this](mavsdk::Telemetry::VtolState vtol_state) {
          switch (vtol_state) {
          case mavsdk::Telemetry::VtolState::Undefined:
            state.vtol_state = custom_msgs::msg::DroneState::UNDEFINED;
            break;
          case mavsdk::Telemetry::VtolState::TransitionToFw:
            state.vtol_state = custom_msgs::msg::DroneState::TRANSITION_TO_FW;
            break;
          case mavsdk::Telemetry::VtolState::TransitionToMc:
            state.vtol_state = custom_msgs::msg::DroneState::TRANSITION_TO_MC;
            break;
          case mavsdk::Telemetry::VtolState::Fw:
            state.vtol_state = custom_msgs::msg::DroneState::FW;
            break;
          case mavsdk::Telemetry::VtolState::Mc:
            state.vtol_state = custom_msgs::msg::DroneState::MC;
            break;
          }
        });

    mavsdk_telem->subscribe_position_velocity_ned(
        [this](mavsdk::Telemetry::PositionVelocityNed pos) {
          state.n = pos.position.north_m;
          state.e = pos.position.east_m;
          state.d = pos.position.down_m;

          state.vn = pos.velocity.north_m_s;
          state.ve = pos.velocity.east_m_s;
          state.vd = pos.velocity.down_m_s;

          transform.transform.translation.x = pos.position.north_m;
          transform.transform.translation.y = pos.position.east_m;
          transform.transform.translation.z = pos.position.down_m;
        });

    auto gps_orig = this->mavsdk_telem->get_gps_global_origin();
    state.origin_lon = gps_orig.second.longitude_deg;
    state.origin_lat = gps_orig.second.latitude_deg;
    state.origin_alt = gps_orig.second.altitude_m;

    mavsdk_telem->subscribe_attitude_quaternion(
        [this](mavsdk::Telemetry::Quaternion quat) {
          state.x = quat.x;
          state.y = quat.y;
          state.z = quat.z;
          state.w = quat.w;
          transform.transform.rotation.x = quat.x;
          transform.transform.rotation.y = quat.y;
          transform.transform.rotation.z = quat.z;
          transform.transform.rotation.w = quat.w;
        });

    mavsdk_telem->subscribe_attitude_angular_velocity_body(
        [this](mavsdk::Telemetry::AngularVelocityBody angular_vel) {
          state.roll_rad_s = angular_vel.roll_rad_s;
          state.pitch_rad_s = angular_vel.pitch_rad_s;
          state.yaw_rad_s = angular_vel.yaw_rad_s;
        });

    mavsdk_telem->subscribe_flight_mode(
        [this](mavsdk::Telemetry::FlightMode fm) {
          switch (fm) {
          case mavsdk::Telemetry::FlightMode::Unknown:
            state.flight_mode = custom_msgs::msg::DroneState::UNKNOWN;
            break;
          case mavsdk::Telemetry::FlightMode::Ready:
            state.flight_mode = custom_msgs::msg::DroneState::READY;
            break;
          case mavsdk::Telemetry::FlightMode::Takeoff:
            state.flight_mode = custom_msgs::msg::DroneState::TAKEOFF;
            break;
          case mavsdk::Telemetry::FlightMode::Hold:
            state.flight_mode = custom_msgs::msg::DroneState::HOLD;
            break;
          case mavsdk::Telemetry::FlightMode::Mission:
            state.flight_mode = custom_msgs::msg::DroneState::MISSION;
            break;
          case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            state.flight_mode = custom_msgs::msg::DroneState::RETURNTOLAUNCH;
            break;
          case mavsdk::Telemetry::FlightMode::Land:
            state.flight_mode = custom_msgs::msg::DroneState::LAND;
            break;
          case mavsdk::Telemetry::FlightMode::Offboard:
            state.flight_mode = custom_msgs::msg::DroneState::OFFBOARD;
            break;
          case mavsdk::Telemetry::FlightMode::FollowMe:
            state.flight_mode = custom_msgs::msg::DroneState::FOLLOWME;
            break;
          case mavsdk::Telemetry::FlightMode::Manual:
            state.flight_mode = custom_msgs::msg::DroneState::MANUAL;
            break;
          case mavsdk::Telemetry::FlightMode::Altctl:
            state.flight_mode = custom_msgs::msg::DroneState::ALTCTL;
            break;
          case mavsdk::Telemetry::FlightMode::Posctl:
            state.flight_mode = custom_msgs::msg::DroneState::POSCTL;
            break;
          case mavsdk::Telemetry::FlightMode::Acro:
            state.flight_mode = custom_msgs::msg::DroneState::ACRO;
            break;
          case mavsdk::Telemetry::FlightMode::Stabilized:
            state.flight_mode = custom_msgs::msg::DroneState::STABILIZED;
            break;
          case mavsdk::Telemetry::FlightMode::Rattitude:
            state.flight_mode = custom_msgs::msg::DroneState::RATTITUDE;
            break;
          }
        });
  }

  void monitoring_callback() {
    state.header.stamp = this->get_clock()->now();
    transform.header.stamp = this->get_clock()->now();
    state.command_queue_size = command_deque.size();
    if (command_deque.size() == 0) {
      state.current_command = "None";
    } else {
      state.current_command = command_deque[0]->get_description();
    }
    this->tf_drone_broadcaster->sendTransform(transform);
    this->monitoring_pub->publish(state);
  }

  // TODO: DO THIS
  void enqueue_topic_callback(const custom_msgs::msg::Action &action) {
    std::unique_ptr<OffboardNode::Command> com;
    switch (action.action_type) {
    case custom_msgs::msg::Action::HOLD:
      // TODO: As in this and

      break;
    case custom_msgs::msg::Action::GOTO_ACTION:
      if (action.vtol_config == custom_msgs::msg::Action::IN_FW) {
        // TODO: this
        RCLCPP_INFO(this->get_logger(), "NOT IMPLEMENTED YET");
        return;
      }
      if (action.vtol_config == custom_msgs::msg::Action::IN_QC) {
        com = std::make_unique<OffboardNode::GoToQC>(action.n, action.e,
                                                     action.d);
      }
      break;
    case custom_msgs::msg::Action::LAND:
      com = std::make_unique<OffboardNode::Land>();
      break;
    case custom_msgs::msg::Action::TAKEOFF_ACTION:
      com = std::make_unique<OffboardNode::TakeOff>(action.takeoff_height);
      break;
    }
    command_deque.push_back(std::move(com));
  }

  void cancel_command_topic_callback(const std_msgs::msg::Empty &em) {
    if (command_deque.size() > 0)
      want_to_cancel = true;
  }

  void command_queue_callback() {
    // RCLCPP_INFO(this->get_logger(), "Command queue has size %d",
    //             command_deque.size());
    if (command_deque.size() == 0) {
      if (state.flight_mode == custom_msgs::msg::DroneState::OFFBOARD) {
        mavsdk_offboard->stop();
      }
      // This may make it impossible for the Remote controller to take over,
      // above should suffice if (state.flight_mode !=
      // custom_msgs::msg::DroneState::HOLD && state.flight_mode !=
      // custom_msgs::msg::DroneState::MANUAL) {
      //   mavsdk_action->hold();
      // }
      return;
    }
    bool is_done = (want_to_cancel) ? command_deque[0]->cancel_command(*this)
                                    : command_deque[0]->perform_tick(*this);

    if (is_done) {
      want_to_cancel = false;
      command_deque.pop_front();
    }
  }

  // flight parameters
  double qc_tolerance;

  // ROS2 related stuff
  rclcpp::Publisher<custom_msgs::msg::DroneState>::SharedPtr monitoring_pub;
  rclcpp::Subscription<custom_msgs::msg::Action>::SharedPtr enqueue_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pop_sub;

  custom_msgs::msg::DroneState state;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  // tf2 - transforms for Rviz visualization
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_world_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_drone_broadcaster;
  geometry_msgs::msg::TransformStamped transform;

  // mavsdk related stuff
  std::optional<std::shared_ptr<mavsdk::System>> sys;
  std::unique_ptr<mavsdk::Mavsdk> mavsdk;
  std::unique_ptr<mavsdk::Telemetry> mavsdk_telem;
  std::unique_ptr<mavsdk::Action> mavsdk_action;
  std::unique_ptr<mavsdk::Offboard> mavsdk_offboard;

  // Command queue
  std::deque<std::unique_ptr<OffboardNode::Command>> command_deque = {};
  bool want_to_cancel = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
