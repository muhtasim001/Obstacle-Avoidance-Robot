#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"


struct mapInfo {
  int size_x_real,size_y_real,inflation_cost,cell_size_x,cell_size_y,index_size,cell_size;
  float inflation_radius,inflation_cost,resolution;
};
 
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
    mapInfo gInfo;
};
 
#endif 
