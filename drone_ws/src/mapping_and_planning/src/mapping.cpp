#include "mapping_and_planning/mapping.hpp"



Planner::Planner()
  : Node("planner"), drone_frame_(4, 4), octomap_granularity_(3.) {
    this->drone_frame_(3, 3) = 1.;
    Quaterniond quat(1, 0, 0, 0);
    auto rotation = quat.matrix();
    Vector3d translation(10, 20, 30);
    this->drone_frame_.block<3, 3>(0, 0) = rotation;
    this->drone_frame_.block<3, 1>(0, 3) = translation;

    bounds_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "mission_boundary", 10, bind(&Planner::boundaryCallback, this, _1));
    
    obs_sub_ = this->create_subscription<mapping_and_planning::msg::Obstacle>(
      "detection", 10, std::bind(&Planner::obstacleCallback, this, _1));
    cout << this->drone_frame_ << endl;

    // Example boundary and detection, refer to the mapping docs for how the map looks like
    vector<Eigen::Vector2d> boundary{{-10, 30}, {20, 30}, {20, 10},
                                          {30, 10},  {30, 0},  {-30, 0},
                                          {-30, 10}, {-10, 10}};
   
    // First detection should be right above the drone, 5 meters away, with radius 2
    // Second on should be to the right of the drone, 5 meters away, with a radius 10.
    vector<Vector4d> detections{{0.0, M_PI, 5., 2.}, {M_PI, 2*M_PI, 20., 10.}};


    octomap::OcTree tree = build_octomap(boundary, detections);
    tree.writeBinary("simple_tree.bt");
    cout << "wrote octomap file simple_tree.bt" << endl << endl;
    cout << "now you can use octovis to visualize: octovis simple_tree.bt"
              << endl;
  }

  void Planner::boundaryCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a polygon with %ld vertices.", msg->points.size());
        boundaries_.resize(msg->points.size());
        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            const auto &point = msg->points[i];
            boundaries_[i] = Vector2d(point.x, point.y);
            RCLCPP_INFO(this->get_logger(), "Vertex %ld: [x=%.2f, y=%.2f, z=%.2f]", i, point.x, point.y, point.z);
        }
    }

  void Planner::obstacleCallback(const mapping_and_planning::msg::ObstacleArray::SharedPtr obs){
    RCLCPP_INFO(this->get_logger(), "Received detections.");
        detections_.resize(obs->obstacles.size());
        detections_ = obs->obstacles;
    }

  octomap::OcTree Planner::build_octomap(std::vector<Eigen::Vector2d> boundary,
                                std::vector<Eigen::Vector4d> detections) {
    octomap::OcTree tree(this->octomap_granularity_);

    // TODO: Fill your code here
    // Build the tree, refer to
    // https://github.com/OctoMap/octomap/blob/devel/octomap/src/simple_example.cpp
    // for an example of how to use octomap The drone's frame of reference is in
    // the this->drone_frame matrix (homogenous coords) std::vector is C++'s
    // class for an ArrayList, Eigen::Vector2d and Eigen::Vector4d are eigen's
    // implementation of linear algebra vectors
    // Avoid the keyword `new` or any form of heap utilization, just use the
    // stack! If you do use `new`, always remember to use `delete` when
    // appropriate to free the memory.
    cout << "Boundary :" << endl;
    for (auto vec : boundary) {
      cout << vec << endl;
    }
    cout << "Detections :" << endl;
    for (auto vec : detections) {
      cout << vec << endl;
    }
    return tree;
  }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
