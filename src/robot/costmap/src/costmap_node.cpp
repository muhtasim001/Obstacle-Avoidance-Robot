#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  //string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  //initalize publishers and subscribers
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
  laider_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10,
                  std::bind(&CostmapNode::laserCallback,this,std::placeholders::_1));

}
 

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

  //step 2
  initializeCostmap();

  message.header = scan->header;

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {

      int gridX = 0, gridY = 0;
      //step 3
      convertToGrid(range,angle,gridX,gridY);
      //step 4
      markObstacle(gridX,gridY);
    }
  }

  //step 5
  inflateObstacles();

  //step 6
  publishCostmap();
  
}

void CostmapNode::initializeCostmap() {
  
  //initalize row
  costMapGrid = new int8_t*[GRID_SIZE];

  //initalize collum and set it to 0
  for (int i = 0; i < GRID_SIZE; i++) {
    costMapGrid[i] = new int8_t[GRID_SIZE];
    for (int j = 0; j < GRID_SIZE; j++)
      costMapGrid[i][j] = 0;
  }
  
}

//sorry I couldn't really figure out the more efficent bfs approch
//(T_T) I am still really new to some of the dsa concpets and need to 
//work on them (T_T)
void CostmapNode::inflateObstacles() {
  //since the inflation radius is 0.8m need to search,
  //also its more of a square than a radius (T_T)
  int inflationCellRadius = INFLATION_RADIUS/GRID_RESOLUTION;

  for (int i = 0 ; i < GRID_SIZE ; i ++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      if (costMapGrid[i][j] == 100) {  
        for (int k = i - inflationCellRadius ; k < i + inflationCellRadius ; k++) {
          for (int l = j - inflationCellRadius; l < j + inflationCellRadius; l++) {
            if (k >= 0 && k < GRID_SIZE && l >= 0 && l < GRID_SIZE && costMapGrid[k][l] < 100) {

              float euclideanDistance = GRID_RESOLUTION * sqrt(pow(((i - k)),2)+ pow((j - l),2));
              int cost = (int)(INFLATION_COST * (1 - (euclideanDistance/INFLATION_RADIUS)));

              if (costMapGrid[k][l] < cost)
                costMapGrid[k][l] = cost;
            }
          }
        }
      }
    }
  }
}

void CostmapNode::markObstacle(int gridX,int grdiY) {
  if (gridX < GRID_SIZE && gridX > 0 && grdiY < GRID_SIZE && grdiY >= 0)
    costMapGrid[gridX][grdiY] = INFLATION_COST;
}

void CostmapNode::convertToGrid(double range,double angle,int &gridX,int &gridY) {
  
  // Compute the Cartesian coordinates of the detected point
  double xCord = range * cos(angle);
  double yCord = range * sin(angle);

  //account for the shift as the robot is in the certer of the 2d array
  gridX = (int)std::round((xCord / GRID_RESOLUTION + GRID_X_SIZE/2));
  gridY = (int)std::round((yCord / GRID_RESOLUTION + GRID_Y_SIZE/2));

}

void CostmapNode::publishCostmap() {
  //the info
  message.info.height = GRID_Y_SIZE;
  message.info.width = GRID_X_SIZE;
  message.info.resolution = GRID_RESOLUTION;
  message.info.origin.position.z = 0;
  message.info.origin.position.x = GRID_SIZE/2.0;
  message.info.origin.position.y = GRID_SIZE/2.0;
  message.info.origin.orientation.w = 1.0;

  //flated the 2d cost map
  int index = 0;
  int8_t myMap [GRID_SIZE*GRID_SIZE] = {};

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      myMap[index] = costMapGrid[i][j];
      index++;
    }
  }

  //free up the 2d array
  for (int i = 0; i < GRID_SIZE; i++) {
    delete [] costMapGrid[i];
  }
  delete [] costMapGrid;

  

  //clamp the map between 0 and 100
  for (int i = 0; i < GRID_X_SIZE * GRID_Y_SIZE ; i++) {
    if (message.data[i] < 0) message.data[i] = 0;
    if (message.data[i] > INFLATION_COST) message.data[i] = 100;
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