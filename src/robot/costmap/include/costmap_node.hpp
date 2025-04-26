#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"

//useful constants 
const int GRID_X_SIZE = 20,GRID_Y_SIZE = 20;
const float GRID_RESOLUTION = 0.1;
const int INFLATION_COST = 100;
const float INFLATION_RADIUS = 0.8;
const int GRID_SIZE = 20 / GRID_RESOLUTION;
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
 
  private:
    robot::CostmapCore costmap_;

    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laider_sub_;  

    rclcpp::TimerBase::SharedPtr timer_;

    //helper functions
    void initializeCostmap();
    void inflateObstacles();
    void publishCostmap();
    void convertToGrid(double range,double angle,int &gridX,int &gridY);
    void markObstacle(int gridX,int grdiY);

    //the cost map grid
    int8_t **costMapGrid;
    nav_msgs::msg::OccupancyGrid message;
};
 
#endif 
