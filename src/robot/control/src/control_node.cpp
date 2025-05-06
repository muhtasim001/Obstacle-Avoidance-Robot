#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {

  //initalize parameters 
  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  linear_speed = 0.5;

  //publishers and subscribers
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
  
  path_sub = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path = msg; });

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom = msg; });

  //timer 
  control_timer = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });

}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path || !robot_odom || current_path->poses.empty()) {
    //RCLCPP_WARN(this->get_logger(),"path is empty, not valid points to use");
    return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();

  if (!lookahead_point) { // since there are no points or the goal has been reached, stop the robot

    geometry_msgs::msg::Twist robot_velo;
    robot_velo.angular.z = 0.0;
    robot_velo.linear.x = 0.0;
    cmd_vel_pub->publish(robot_velo);

    RCLCPP_WARN(this->get_logger(),"no valid look ahead point found");
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // find the current robot position
  auto current_spot = robot_odom->pose.pose.position;

  //return null if the robot distance is within gloal tolerence
  if (computeDistance(current_spot,current_path->poses.back().pose.position) < goal_tolerance) {
    return std::nullopt;
    RCLCPP_INFO(this->get_logger(),"reached the goal");
  }
  
  //loop though the points, cheeking untill a point in geather than or equal to look ahead 
  for (const auto &lookAheadPoint:current_path->poses) {
    if (computeDistance(current_spot,lookAheadPoint.pose.position) >= lookahead_distance) {
      RCLCPP_WARN(this->get_logger(),
      "the look ahead point x = %2.0f and y = %2.0f",
      lookAheadPoint.pose.position.x,lookAheadPoint.pose.position.y);
      
      return lookAheadPoint;
    }
  }
  
  //all fails, and we don't find one, than return null
  return std::nullopt;  
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  const auto robot_current = robot_odom->pose.pose;

  //figure out the heading angle 
  double heading = std::atan2(target.pose.position.y - robot_current.position.y,target.pose.position.x - robot_current.position.x);

  //figure out the robot current diretion
  double current_angle = extractYaw(robot_current.orientation);

  //find the error between the heading to find angular velocity
  double heading_error = heading - current_angle;

  //normalize error to between -pi and pi
  while (heading_error > M_PI ) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  //generate the velocity comand
  cmd_vel.angular.z = 2.0 * heading_error;
  cmd_vel.linear.x = linear_speed;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return sqrt(pow(b.x - a.x,2) + pow(b.y - a.y,2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2.0f * (quat.w * quat.z + quat.x * quat.y),std::pow(quat.w,2) + std::pow(quat.x,2) - std::pow(quat.y,2) - std::pow(quat.z,2));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
