#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "Eigen/Dense"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "mapping_and_planning/msg/obstacle.hpp"
#include "mapping_and_planning/msg/obstacle_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class Planner : public rclcpp::Node{
public:
    Planner();
    octomap::OcTree build_octomap(std::vector<Vector2d> boundary,
                                std::vector<Vector4d> detections);
private:
    void boundaryCallback(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void obstacleCallback(const mapping_and_planning::msg::ObstacleArray::SharedPtr obs);
    vector<mapping_and_planning::msg::Obstacle> detections_;
    vector<Vector2d> boundaries_;
    MatrixXd drone_frame_;
    double octomap_granularity_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr bounds_sub_;
    rclcpp::Subscription<mapping_and_planning::msg::ObstacleArray>::SharedPtr obs_sub_;
};

#endif  // MAPPING_HPP