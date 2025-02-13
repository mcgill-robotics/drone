#include <cmath>
#include <cstdio>
#include <custom_msgs/msg/detail/action__struct.hpp>
#include <custom_msgs/msg/detail/drone_state__struct.hpp>
#include <deque>
#include <functional>
#include <iterator>
#include <math.h>
#include <mavlink/common/mavlink.h>
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

// UTILITY FUNCTIONS
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

/* Returns the smallest angle between 2 angles, in degrees.
 * */
double smallestAngle(double currentAngle, double targetAngle) {
  double bigger = std::max(currentAngle, targetAngle);
  double smaller = std::min(currentAngle, targetAngle);
  return std::min(bigger - smaller, smaller + 360. - bigger);
}

class OffboardNode : public rclcpp::Node {
public:
  // TODO: Add parameters as parameters
  OffboardNode() : Node("offboard_controller"), qc_tolerance(1.) {

    // tf2 stuff
    tf_world_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->setup_world_frame();

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

    // TODO: DECOMMISSIONED FOR NOW, COME BACK TO THIS LATER
    // pop_sub = this->create_subscription<std_msgs::msg::Empty>(
    //     "cancel_command", 10,
    //     std::bind(&OffboardNode::cancel_command_topic_callback, this,
    //               std::placeholders::_1));

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&OffboardNode::monitoring_callback, this));

    this->command_timer_ = this->create_wall_timer(
        100ms, std::bind(&OffboardNode::command_queue_callback, this));
  }

private:
  /* Objects that represent a command. This defines a common interface and has
   * utility functions
   * */
  struct Command {
    // both perform tick and cancel_command return true if they are done, false
    // otherwise

    /* Performs one tick of the command. This is for the command to be
     * non-blocking
     * */
    virtual bool perform_tick(OffboardNode &on) = 0;
    /* This is also one tick for the same reason as above, but is supposed to
     * handel cancelations properly. FOR NOW, DO NOT DO THIS (OUT OF SCOPE)
     * */
    virtual bool cancel_command(OffboardNode &on) {
      RCLCPP_INFO(on.get_logger(), "This command is not cancelable");
      return perform_tick(on);
    }
    /* Returns a string that describes the command. For debugging purposes.
     * */
    virtual std::string get_description() = 0;

    /* Function that transforms the this command to adapt to the new_command
     *  This is really only relevant for FW commands as they need to change
     * behavour if the new_command is also FW (ex: need to stay in FW and not
     * transition after the goal is reached)
     * */
    virtual void mutate(Command *new_command) {}
    virtual ~Command() {}

  protected:
    /* Sends a way point command, over 2 ticks if not in offboard mode, over 1
     * tick otherwise. returns true when sending is done (not when reached
     * destination) If you don't want control over a parameter (like yaw), set
     * it to NAN
     * */
    bool send_offboard_goto(OffboardNode &node, double north, double east,
                            double down, double yaw) {
      mavsdk::Offboard::PositionNedYaw pos;
      pos.north_m = north;
      pos.east_m = east;
      pos.down_m = down;
      pos.yaw_deg = yaw;

      auto result = node.mavsdk_offboard->set_position_ned(pos);
      if (node.state.flight_mode != custom_msgs::msg::DroneState::OFFBOARD) {
        node.mavsdk_offboard->start();
        return false;
      }
      return true;
    }

    double distance_from_target(OffboardNode &node, double target_n,
                                double target_e, double target_d) {
      return sqrt(pow((node.state.n - target_n), 2) +
                  pow((node.state.e - target_e), 2) +
                  pow((node.state.d - target_d), 2));
    }
  };

  /* Command responsible for taking off to a certain height.
   * */
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

  /* Command responsible for landing.
   * */
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

  /* Command responsible for going to a destination in QC mode.
   * */
  struct GoToQC : Command {
    GoToQC(double n, double e, double d)
        : target_n(n), target_e(e), target_d(d) {}

    virtual bool perform_tick(OffboardNode &node) override {
      double dist_from_target =
          distance_from_target(node, target_n, target_e, target_d);
      if (dist_from_target >= node.qc_tolerance && !goto_command_sent) {
        goto_command_sent =
            send_offboard_goto(node, target_n, target_e, target_d, NAN);
      }

      return dist_from_target < node.qc_tolerance;
    }

    virtual std::string get_description() override {
      return string_format("Going to %lf, %lf, %lf in QC", target_n, target_e,
                           target_d);
    }

    virtual bool cancel_command(OffboardNode &node) override { return true; }

    ~GoToQC() {}
    double target_n, target_e, target_d;
    bool goto_command_sent = false;
  };

  /* Command responsible for going to a destination in FW mode. This needs to
   * adapt it's behabiour w.r.t. the next command. All new commands start as
   * SOLE (alone). They then transition to the appropriate type of FW Command if
   * need to.
   * */
  struct GoToFW : Command {

    GoToFW(double n, double e, double d)
        : target_n(n), target_e(e), target_d(d) {}

    virtual bool perform_tick(OffboardNode &node) override {
      bool res;
      switch (fw_command_type) {
      case TypeFW::SOLE:
        res = sole_flight(node);
        break;
      case TypeFW::STARTING_FW:
        res = starting_fw_flight(node);
        break;
      case TypeFW::MIDDLE_FW:
        res = middle_fw_flight(node);
        break;
      case TypeFW::END_FW:
        res = end_fw_flight(node);
        break;
      }
      return res;
    }

    virtual std::string get_description() override {
      return string_format(
          "Going to %lf, %lf, %lf in FW (FW Type: %s | FW Stage %s)", target_n,
          target_e, target_d, get_type_str().c_str(),
          get_lifetime_str().c_str());
    }

    virtual void mutate(Command *new_command) override {

      GoToFW *command_ptr = dynamic_cast<GoToFW *>(new_command);
      if (command_ptr) {
        command_ptr->adapt();
        switch (fw_command_type) {
        case TypeFW::SOLE:
          fw_command_type = TypeFW::STARTING_FW;
          break;
        case TypeFW::STARTING_FW:
          std::cout << "THIS COMMAD IS STARTING_FW AND AT THE END OF THE "
                       "QUEUE, IMPOSSIBLE!! EXITING..."
                    << std::endl;
          exit(2);
          break;
        case TypeFW::MIDDLE_FW:
          std::cout << "THIS COMMAD IS MIDDLE_FW AND AT THE END OF THE "
                       "QUEUE, IMPOSSIBLE!! EXITING..."
                    << std::endl;
          exit(3);
          break;
        case TypeFW::END_FW:
          fw_command_type = TypeFW::MIDDLE_FW;
          break;
        }
      }
    }

  private:
    /* Describes the different types of FW commands in a command. Very much like
     * POS Tagging START_FW: Start of a sequence of FW commands, responsible for
     * starting the sequence (fix heading and transition) and reaching the
     * destination. MIDDLE_FW: Middle of a sequence of FW commands, responsible
     * for stopping when destination is reached. END_FW: End of a sequence of FW
     * commands, responsible for untransitionning slightly before reaching the
     * destination, then stop when destination is reached. SOLE: Unique FW
     * command, responsible for both parts of STARTING_FW behaviour and END_FW
     * behaviour at the start and end respectively.
     * */
    enum class TypeFW { SOLE, STARTING_FW, MIDDLE_FW, END_FW };
    std::string get_type_str() {
      switch (fw_command_type) {
      case TypeFW::SOLE:
        return "SOLE";
      case TypeFW::STARTING_FW:
        return "START";
      case TypeFW::MIDDLE_FW:
        return "MIDDLE";
      case TypeFW::END_FW:
        return "END";
      }
    }
    /* Make this command the end of a FW sequence, only called if previous
     * command is FW and gets mutated
     * */
    void adapt() { fw_command_type = TypeFW::END_FW; }

    /* Behaviour needs to once again change w.r.t. what's been done already
     * A task should only be complete once its done with its STOP stage.
     * */
    enum class LifetimeStage { BEGIN, ONGOING, STOP };
    std::string get_lifetime_str() {
      switch (current_stage) {
      case LifetimeStage::BEGIN:
        return "BEGIN";
      case LifetimeStage::ONGOING:
        return "ONGOING";
      case LifetimeStage::STOP:
        return "STOP";
      }
    }

    /* A function which alligns the heading of the drone with the target
     * destination returns true when at the appropriate heading only.
     * */
    bool fix_heading(OffboardNode &node) {
      double desired_heading =
          std::atan2(target_e - node.state.e, target_n - node.state.n) * 180. /
          M_PI;
      desired_heading =
          (desired_heading >= 0) ? desired_heading : 360. + desired_heading;
      double current_heading = node.state.yaw_deg;
      std::cout << "Desired : " << desired_heading
                << ", Current : " << current_heading << std::endl;
      if (smallestAngle(desired_heading, current_heading) < 5.0) {
        return true;
      }

      if (!yaw_fixing_command_sent) {
        yaw_fixing_command_sent = send_offboard_goto(
            node, node.state.n, node.state.e, node.state.d, desired_heading);
      }

      return false;
    }

    bool send_waypoint(OffboardNode &node) {
      if (!goto_command_sent) {
        goto_command_sent =
            send_offboard_goto(node, target_n, target_e, target_d, NAN);
      }
      return goto_command_sent;
    }

    /* A function which sends the waypoint and transition commands. Returns true
     * when those commands are successfully sent
     * */
    bool send_waypoint_transition(OffboardNode &node) {
      goto_command_sent = send_waypoint(node);
      double current_speed =
          sqrt(pow(node.state.vn, 2) + pow(node.state.ve, 2));
      if (current_speed >= 5. && goto_command_sent &&
          !transition_command_sent) {
        auto res = node.mavsdk_action->transition_to_fixedwing();
        transition_command_sent =
            (res == mavsdk::Action::Result::Success) &&
            node.state.vtol_state == custom_msgs::msg::DroneState::FW;
      }
      return goto_command_sent && transition_command_sent;
    }

    /* A function which sends a untransition command. Returns true when the
     * command is successful.
     * */
    bool send_untransition(OffboardNode &node) {
      return node.mavsdk_action->transition_to_multicopter() ==
             mavsdk::Action::Result::Success;
    }

    bool is_within_from_target(OffboardNode &node, double meters) {
      return distance_from_target(node, target_n, target_e, target_d) < meters;
    }

    bool sole_flight(OffboardNode &node) {
      // FIX HEADING
      // ONCE FIXED SET WAYPOINT THEN TRANSITION
      // THEN PRE-UNTRANSITION BEFORE REACING END
      // BOOL -> RETURN TRUE WHEN IN QC AND REACHED
      switch (current_stage) {
      case LifetimeStage::BEGIN:
        if (fix_heading(node))
          current_stage = LifetimeStage::ONGOING;
        return false;
        break;
      case LifetimeStage::ONGOING:
        if (send_waypoint_transition(node))
          current_stage = LifetimeStage::STOP;
        return false;
        break;
      case LifetimeStage::STOP:
        if (is_within_from_target(node, 50.) && !untransition_command_sent) {
          untransition_command_sent = send_untransition(node);
          goto_command_sent = false;
          goto_command_sent = send_waypoint(node);
        }
        if (is_within_from_target(node, 5.) && untransition_command_sent)
          return true;
        return false;
        break;
      }
    }

    bool starting_fw_flight(OffboardNode &node) {
      // FIX HEADING
      // ONCE FIXED SET WAYPOINT THEN TRANSITION
      // BOOL -> RETURN TRUE WHEN REACHED
      switch (current_stage) {
      case LifetimeStage::BEGIN:
        if (fix_heading(node))
          current_stage = LifetimeStage::ONGOING;
        return false;
        break;
      case LifetimeStage::ONGOING:
        if (send_waypoint_transition(node))
          current_stage = LifetimeStage::STOP;
        return false;
        break;
      case LifetimeStage::STOP:
        // if () return true;
        if (is_within_from_target(node, 5.))
          return true;
        return false;
        break;
      }
    }

    bool middle_fw_flight(OffboardNode &node) {
      // SET WAYPOINT
      // BOOL -> RETURN TRUE WHEN REACHED
      switch (current_stage) {
      case LifetimeStage::BEGIN:
        current_stage = LifetimeStage::ONGOING;
        return false;
        break;
      case LifetimeStage::ONGOING:
        if (send_waypoint(node))
          current_stage = LifetimeStage::STOP;
        return false;
        break;
      case LifetimeStage::STOP:
        // if () return true;
        if (is_within_from_target(node, 5.))
          return true;
        return false;
        break;
      }
      return false;
    }

    bool end_fw_flight(OffboardNode &node) {
      // SET WAYPOINT
      // THEN PRE-UNTRANSITION BEFORE REACING END
      // BOOL -> RETURN TRUE WHEN IN QC AND REACHED
      switch (current_stage) {
      case LifetimeStage::BEGIN:
        current_stage = LifetimeStage::ONGOING;
        return false;
        break;
      case LifetimeStage::ONGOING:
        if (send_waypoint(node))
          current_stage = LifetimeStage::STOP;
        return false;
        break;
      case LifetimeStage::STOP:
        if (is_within_from_target(node, 50.) && !untransition_command_sent) {
          untransition_command_sent = send_untransition(node);
          goto_command_sent = false;
          goto_command_sent = send_waypoint(node);
        }
        if (is_within_from_target(node, 5.) && untransition_command_sent)
          return true;
        return false;
        break;
      }
      return false;
    }

    TypeFW fw_command_type = TypeFW::SOLE;
    LifetimeStage current_stage = LifetimeStage::BEGIN;
    double target_n, target_e, target_d;

    // FLAGS
    bool yaw_fixing_command_sent = false;
    bool goto_command_sent = false;
    bool transition_command_sent = false;
    bool untransition_command_sent = false;
  };

  // TODO: IMPLEMENT HOLD
  struct Hold : Command {};

  /* A queue composed object, needed to automatically call mutate on append.
   * */
  struct CommandQueue {

    CommandQueue() {}

    int size() { return command_queue.size(); }

    Command *operator[](std::size_t idx) { return command_queue[idx].get(); }

    void push_back(std::unique_ptr<Command> c) {
      command_queue.push_back(std::move(c));
    }

    void enqueue_command(std::unique_ptr<Command> c) {
      if (size() > 0)
        command_queue[size() - 1]->mutate(c.get());
      push_back(std::move(c));
    }

    void pop_front() { command_queue.pop_front(); }

  private:
    std::deque<std::unique_ptr<Command>> command_queue = {};
  };

  /* Sets up the world_frame object (used by RViz).
   * */
  void setup_world_frame() {
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

  /* Sets up the various mavsdk callbacks necessary to update the state and
   * world_frame.
   * */
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

    mavsdk_telem->subscribe_attitude_euler(
        [this](mavsdk::Telemetry::EulerAngle angles) {
          state.roll_deg = angles.roll_deg;
          state.pitch_deg = angles.pitch_deg;
          state.yaw_deg = angles.yaw_deg;
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

  /* Callback to publish the state and world_frame
   * */
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

  /* Callback to enqueue a new topic in the topic queue.
   * */
  // TODO: DO THIS
  void enqueue_topic_callback(const custom_msgs::msg::Action &action) {
    std::unique_ptr<OffboardNode::Command> com;
    switch (action.action_type) {
    case custom_msgs::msg::Action::HOLD:
      // TODO: As in this and

      break;
    case custom_msgs::msg::Action::GOTO_ACTION:
      if (action.vtol_config == custom_msgs::msg::Action::IN_FW) {
        com = std::make_unique<OffboardNode::GoToFW>(action.n, action.e,
                                                     action.d);
      } else if (action.vtol_config == custom_msgs::msg::Action::IN_QC) {
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
    default:
      std::cout << "ACTION TYPE INVALID!! EXITING..." << std::endl;
      exit(1);
      break;
    }
    command_deque.enqueue_command(std::move(com));
  }

  // TODO: DECOMMISIONED, COME BACK TO THIS LATER
  // void cancel_command_topic_callback(const std_msgs::msg::Empty &em) {
  //   if (command_deque.size() > 0)
  //     want_to_cancel = true;
  // }

  /* Responsible for running one tick of the top command.
   * */
  void command_queue_callback() {
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
    // bool is_done = (want_to_cancel) ? command_deque[0]->cancel_command(*this)
    //                                 : command_deque[0]->perform_tick(*this);

    bool is_done = command_deque[0]->perform_tick(*this);
    if (is_done) {
      // want_to_cancel = false;
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
  CommandQueue command_deque = {};
  // bool want_to_cancel = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
