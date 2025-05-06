#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  //initalize some variables
  gInfo.size_x_real = 15; // in meters
  gInfo.size_y_real = 15; // in meters
  gInfo.resolution = 0.1; 
  gInfo.cell_size_x = 15 / 0.1; // size x / resoltion
  gInfo.cell_size_y = 15 / 0.1; // size y / resolution
  gInfo.cell_size = 15 / 0.1;
  gInfo.inflation_cost = 100; 
  gInfo.inflation_radius =  1.2; // in meters

  //initalize publishers and subscribers
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
  laider_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
  ("/lidar",10, std::bind(&CostmapNode::laserCallback,this,std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(),"costmap node consturctor sucess");
}
 

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

  //step 2
  CostmapNode::initializeCostmap();

  message.header = scan->header;

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {

      int gridX = 0, gridY = 0;
      //step 3
      CostmapNode::convertToGrid(range,angle,gridX,gridY);
      //step 4
      CostmapNode::markObstacle(gridX,gridY);
    }
  }

  //step 5
  CostmapNode::inflateObstacles();

  //step 5
  CostmapNode::publishCostmap();
  
}

void CostmapNode::initializeCostmap() {
  
  //initalize row
  costMapGrid = new int8_t*[gInfo.cell_size];

  //initalize collum and set it to 0
  for (int i = 0; i < gInfo.cell_size; i++) {
    costMapGrid[i] = new int8_t[gInfo.cell_size];
    for (int j = 0; j < gInfo.cell_size; j++)
      costMapGrid[i][j] = 0;
  }
  
}

//sorry I couldn't really figure out the more efficent bfs approch
//(T_T) I am still really new to some of the dsa concpets and need to 
//work on them (T_T)
void CostmapNode::inflateObstacles() {
  //since the inflation radius is 0.8m need to search,
  //also its more of a square than a radius (T_T)
  int inflationCellRadius = gInfo.inflation_radius/gInfo.resolution;
  

  for (int i = 0 ; i < gInfo.cell_size ; i ++) {
    for (int j = 0; j < gInfo.cell_size; j++) {
      if (costMapGrid[i][j] == 100) {  
        for (int k = i - inflationCellRadius ; k < i + inflationCellRadius ; k++) {
          for (int l = j - inflationCellRadius; l < j + inflationCellRadius; l++) {
            if (k >= 0 && k < gInfo.cell_size && l >= 0 && l < gInfo.cell_size && costMapGrid[k][l] < 100) {

              float euclideanDistance = gInfo.resolution * sqrt(pow(((i - k)),2)+ pow((j - l),2));
              int cost = (int)(gInfo.inflation_cost * (1 - (euclideanDistance/gInfo.inflation_radius)));

              if (costMapGrid[k][l] < cost)
                costMapGrid[k][l] = cost;
            }
          }
        }
      }
    }
  }
}

void CostmapNode::markObstacle(int gridX,int gridY) {
  if (gridX < gInfo.cell_size && gridX > 0 && gridY < gInfo.cell_size && gridY >= 0) {
    costMapGrid[gridY][gridX] = gInfo.inflation_cost;
  }
}

void CostmapNode::convertToGrid(double range,double angle,int &gridX,int &gridY) {
  
  // Compute the Cartesian coordinates of the detected point
  double xCord = range * cos(angle);
  double yCord = range * sin(angle);

  //account for the shift as the robot is in the certer of the 2d array
  gridX = (int)std::round ((xCord/gInfo.resolution + gInfo.cell_size_x/2));
  gridY = (int)std::round ((yCord/gInfo.resolution + gInfo.cell_size_y/2));

}

void CostmapNode::publishCostmap() {
  //the info
  message.info.height = gInfo.cell_size_y;
  message.info.width = gInfo.cell_size_x;
  message.info.resolution = gInfo.resolution;
  message.info.origin.position.z = 0;
  message.info.origin.position.x = -gInfo.size_x_real/2.0;
  message.info.origin.position.y = -gInfo.size_y_real/2.0;
  message.info.origin.orientation.w = 1.0;
  message.data.assign(gInfo.cell_size_x * gInfo.cell_size_y,-1);


  //transfer the message from the 2d to 1d array
  for (int y = 0; y < gInfo.cell_size_y; y++) {
    for (int x = 0; x < gInfo.cell_size_x; x++) {
       int index = y * gInfo.cell_size_x + x;
       message.data[index] = costMapGrid[y][x];
    }
  }
  
  //free up the memory of 2d array
  for (int i = 0; i < gInfo.cell_size_x; i++) {
    delete [] costMapGrid[i];
  }
  delete [] costMapGrid;


  //clamp the map between 0 and 100
  for (int i = 0; i < gInfo.cell_size_x * gInfo.cell_size_y ; i++) {
    if (message.data[i] < 0) message.data[i] = 0;
    if (message.data[i] > gInfo.inflation_cost) message.data[i] = 100;
  }

  //publish the cost map
  costmap_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}