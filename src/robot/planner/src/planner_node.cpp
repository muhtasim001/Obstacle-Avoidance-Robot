#include <chrono>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  //initalize the publishers and subscribers
  path_pub = this->create_publisher<nav_msgs::msg::Path>("/path",10);

  global_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallBack, this, std::placeholders::_1));

  goal_point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallBack, this, std::placeholders::_1));

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallBack, this, std::placeholders::_1));

  //timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));

}

void PlannerNode::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  global_map = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    PlannerNode::planPath();
  }
}

void PlannerNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose = msg->pose.pose;  
}

void PlannerNode::goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {

  goal_point = *msg;
  goal_received = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  PlannerNode::planPath();
}

void PlannerNode::timerCallBack() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      PlannerNode::planPath();
    }
  }
}

bool PlannerNode::goalReached() {

  double dx = goal_point.point.x - robot_pose.position.x;
  double dy = goal_point.point.y - robot_pose.position.y;

  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath() {

  if (!goal_received || global_map.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  // A* Implementation (pseudo-code)
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  // Compute path using A* on current_map_
  
  // Fill path.poses with the resulting waypoints.

  path_pub->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
