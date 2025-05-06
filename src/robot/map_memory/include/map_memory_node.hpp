#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

//ros2 core library
#include "rclcpp/rclcpp.hpp"

//the memory core
#include "map_memory_core.hpp"

//message types
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

//global variables
namespace GM_Var {
  const int x_size = 300;
  const int y_size = 300;
  const float resolution = 0.1f;
  const int gg_size = 300;
  const float gg_ofset = 30/2;
}

struct transformation_var {
  double x,y;
  double qx,qy,qz,qw;
  double theta;
  int ofset;
};

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    //ros2 publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //global map and robot postion
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    double last_x, last_y;
    const double distance_threshold = 1.5;
    bool costmap_updated_ = false;
    transformation_var tv;

    //current global map
    std::vector<std::vector<int8_t>> global_map_2d;


    //call back funtions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //helper and sub functions
    void updateMap();
    void integrateCostmap();
    void convertRobotToWorldIndex(int robot_x,int robot_y,int &world_x,int &world_y);

    //flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
};

#endif 
