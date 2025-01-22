#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "Eigen/Dense"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace octomap;
using namespace Eigen;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

void print_query_info(point3d query, OcTreeNode *node) {
  if (node != NULL) {
    std::cout << "occupancy probability at " << query << ":\t "
              << node->getOccupancy() << std::endl;
  } else
    std::cout << "occupancy probability at " << query << ":\t is unknown"
              << std::endl;
}

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    OcTree tree(0.1); // create empty tree with resolution 0.1
    for (int x = -20; x < 20; x++) {
      for (int y = -20; y < 20; y++) {
        for (int z = -20; z < 20; z++) {
          point3d endpoint((float)x * 0.05f, (float)y * 0.05f,
                           (float)z * 0.05f);
          tree.updateNode(endpoint, true); // integrate 'occupied' measurement
        }
      }
    }

    for (int x = -30; x < 30; x++) {
      for (int y = -30; y < 30; y++) {
        for (int z = -30; z < 30; z++) {
          point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f,
                           (float)z * 0.02f - 1.0f);
          tree.updateNode(endpoint, false); // integrate 'free' measurement
        }
      }
    }

    point3d query(0., 0., 0.);
    OcTreeNode *result = tree.search(query);
    print_query_info(query, result);

    query = point3d(-1., -1., -1.);
    result = tree.search(query);
    print_query_info(query, result);

    query = point3d(1., 1., 1.);
    result = tree.search(query);
    print_query_info(query, result);

    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl;
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
