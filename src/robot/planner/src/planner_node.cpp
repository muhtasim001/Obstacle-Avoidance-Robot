#include <chrono>
#include <memory>
#include <queue>

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

  gInfo.num_col = global_map.info.width/global_map.info.resolution;
  gInfo.num_row = global_map.info.height/global_map.info.resolution;

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

void PlannerNode::aStartPathFinder(CellIndex start,CellIndex target,nav_msgs::msg::Path &path) {

  std::unordered_map<CellIndex, int, CellIndexHash> cell_gCost;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> child_parent;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;
  std::priority_queue<AStarNode, std::vector<AStarNode>,CompareF> open;

  AStarNode startNode (start,hCostCalc(start,target) + 0);
  AStarNode endNode (target, 0 + gCostCalc(target,start));

  open.push(startNode);
  cell_gCost.emplace(startNode.index, 0);

  while (!open.empty()) {

    AStarNode current = open.top();
    open.pop();
    closed.emplace(current.index,true);

    if (current.index == endNode.index) {
      path.poses = reConstructPath (child_parent,startNode,endNode);
      return;
    }

    for (CellIndex myNode:PlannerNode::findNeighbors(current)) {

      if (global_map.data[myNode.y * gInfo.num_col + myNode.x] > 0 || closed.find(myNode) != closed.end()) {
        continue;
      }

      int gCostFromCurrent = cell_gCost[current.index] + gCostCalc(myNode,current.index);

      if (cell_gCost.find(myNode) == cell_gCost.end() || gCostFromCurrent < cell_gCost.at(myNode)) {

        int node_fCost = gCostFromCurrent + hCostCalc(myNode,endNode.index);
        open.push(AStarNode(myNode,node_fCost));

        if (child_parent.find(myNode) != child_parent.end()) {
          child_parent[myNode] = current.index;
        } else {
          child_parent.emplace(myNode,current.index);
        }

        if (cell_gCost.find(myNode) != cell_gCost.end()) {
          cell_gCost[myNode] = gCostFromCurrent;
        } else {
          cell_gCost.emplace(myNode,gCostFromCurrent);
        }

      }

    }

  }

}

int PlannerNode::hCostCalc (CellIndex current,CellIndex start) {
  return (int)((global_map.info.width/global_map.info.resolution)*(abs(current.x - start.x) + abs (current.y - start.y)));
}

int PlannerNode::gCostCalc (CellIndex current,CellIndex goal) {
  return (int)((global_map.info.width/global_map.info.resolution)*(abs(current.x - goal.x) + abs (current.y - goal.y)));
}

std::vector<CellIndex> PlannerNode::findNeighbors(AStarNode current) {

  std::vector<CellIndex> neighbors = {};
  
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      if (x != 0 && y != 0) {
        int cellX = current.index.x + x;
        int cellY = current.index.y + y;

        if (cellX >= 0 && cellX < gInfo.num_col && cellY >= 0 && cellY < gInfo.num_row) {
          CellIndex myCell(cellX,cellY);
          neighbors.push_back(myCell);
        }
      }
    }
  }

  return neighbors;

}

std::vector<geometry_msgs::msg::PoseStamped> PlannerNode::reConstructPath(
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> backTrackList,AStarNode start,AStarNode end) {

  std::vector<geometry_msgs::msg::PoseStamped> pathPoints = {};

  CellIndex curent = end.index;

  geometry_msgs::msg::PoseStamped currentPoint;

  while (curent != start.index) {
    currentPoint.pose.position.x = curent.x/global_map.info.resolution + global_map.info.width/2;
    currentPoint.pose.position.y = curent.y/global_map.info.resolution + global_map.info.height/2;
    currentPoint.pose.position.z = 0;
    
    pathPoints.push_back(currentPoint);

    curent = backTrackList.at(curent);
  }

  std::reverse(pathPoints.begin(),pathPoints.end());

  return pathPoints;

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

  //find the start point
  int robot_start_x = (robot_pose.position.x - ((global_map.info.width*global_map.info.resolution)/2)) * global_map.info.resolution;
  int robot_start_y = (robot_pose.position.y - ((global_map.info.height*global_map.info.resolution)/2)) * global_map.info.resolution;
  CellIndex startPoint (robot_start_x,robot_start_y);
  
  //find the end point
  int robot_end_x = ( goal_point.point.x - ((global_map.info.width*global_map.info.resolution)/2)) * global_map.info.resolution;
  int robot_end_y = ( goal_point.point.y - ((global_map.info.height*global_map.info.resolution)/2)) * global_map.info.resolution;
  CellIndex endPoint (robot_end_x,robot_end_y);
  
  //find the path using the A* 
  PlannerNode::aStartPathFinder(startPoint,endPoint,path);

  path_pub->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
