#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

//message includes 
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;

    //publishers 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    //subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub;

    //timer
    rclcpp::TimerBase::SharedPtr timer_;

    //data storage
    geometry_msgs::msg::PointStamped goal_point;
    nav_msgs::msg::OccupancyGrid global_map;
    geometry_msgs::msg::Pose robot_pose;

    //state
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;
    bool goal_received = false;

    //callback and helper funtions
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallBack();
    bool goalReached();
    void planPath();

    //A* parts

    struct CellIndex {
      int x;
      int y;
 
      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}
 
      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }
 
      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }
    };

    // Hash function for CellIndex so it can be used in std::unordered_map
    struct CellIndexHash {
      std::size_t operator()(const CellIndex &idx) const
      {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
      }
    };
    struct AStarNode
    {
      CellIndex index;
      double f_score;  // f = g + h
 
      AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
    };

    struct CompareF
    {
      bool operator()(const AStarNode &a, const AStarNode &b)
      {
        // We want the node with the smallest f_score on top
        return a.f_score > b.f_score;
      }
    };

};

#endif 

/*
outside the funtion in class private : 
  unordered_map<CellIndex, cellIndex> child_parent;
  unordered_map<cellIndex, int> cell_gCost;

  pathfind(cellIndex starting,CellIndex ending) {

    parent_child.clear();
    cell_gCost.clear();

    AStarNode startNode (staring cellIndex,hCostCalc(starting,ending) + gCostCals(starting,ending));
    AStartNode endNode (ending cellIndex,hCostCalc(ending,ending) + gCostCals(ending,ending));

    priority_que<AStarNode> open;
    unordered_map <cellIndex,bool> closed;

    open.push(startNode);
    cell_gCost.insert(starting,gCostCalc(starting,ending));

    while (!open.empty()) {
      AstartNode current = startNode;
      open.pop();
      closed.insert(current.index,true)

      if (current.index == endNode.index)
        return;
      
      for (cellIndex node:findNeighbors(current)) {

        if (2dGrid[node.y][node.x] > 0 || closed.containes(node)) continue;

        int movemnetCostToNode = cell_gCost.at(current.index) + gCostCalc(current.index,node);
        
        if (movemnetCostToNode < gCostCalc(node,starting) || cel_gCost.contains(node)) {

          int node_f_Cost = movemnetCostToNode + hCostCalc(node,ending);
          child_parent.insert(node,current.index);

          if (!cel_gCost.contains(node)) {
            cell_gCost.insert(node,movemnetCostToNode);
            open.push(AStarNode(node,node_f_cost));
          }
        }

      }
    }
  }

  int hCostCalc (cellIndex current,cellIndex goal) {
    return gridSize*(abs(current.x - goal.x) + abs (current.y - goal.y));
  }

  int gCostCalc (cellIndex current,cellIndex start) {
    return gridSize*(abs(start.x - current.x) + abs(start.y - current.y));
  }

  std::vector<cellIndex> findNeighbors(AStartNode current) {
    std::vector nList;
    //make a loop that makes and adds in a 3x3 radius, skips current
    //also add the node to the parent_child with current as parent and child at the index
    return nList;
  }
*/
