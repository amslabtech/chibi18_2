#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

geometry_msgs::PointStamped start;
geometry_msgs::PointStamped goal;

nav_msgs::OccupancyGrid map;
nav_msgs::Path global_path;

class Grid
{
public:
  Grid(void)
  {
    is_wall = false;
    cost = 1;
  }

  int cost;
  int step;
  int heuristic;
  bool is_wall;
};

std::vector<std::vector<Grid> > cost_map;

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
}

int get_grid_data(float, float);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("START_X", start.point.x);
  local_nh.getParam("START_Y", start.point.y);
  local_nh.getParam("GOAL_X", goal.point.x);
  local_nh.getParam("GOALY", goal.point.y);
 
  start.header.frame_id = "map";
  goal.header.frame_id = "map";

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100);

  ros::Rate loop_rate(10);

  global_path.header.frame_id = "map";

  cost_map.resize(map.info.height);
  for(int i=0;i<map.info.width;i++){
    cost_map[i].resize(map.info.width);
  }
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      cost_map[i][j].is_wall = (map.data[map.info.height*i+j]!=0);
    }
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation.w = 1;
  global_path.poses.push_back(pose);
  pose.pose.position.x = 1;
  global_path.poses.push_back(pose);

  while(ros::ok()){
    if(!map.data.empty()){
      //std::cout << map.data.size() << std::endl;
      float x = -1.48;
      float y = -1.02;
      std::cout << get_grid_data(x, y) << std::endl;
    }
    if(!global_path.poses.empty()){
      path_pub.publish(global_path);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int get_grid_data(float x, float y)
{
  if(x > 0){
    x = ((int)(10*(2*x)+1))/20.0;
  }else{
    x = ((int)(10*(2*x)))/20.0;
  }
  if(y > 0){
    y = ((int)(10*(2*y)))/20.0;
  }else{
    y = ((int)(10*(2*y)-1))/20.0;
  }
  std::cout << x << ", " << y << std::endl;
  int data = map.data[int((map.info.width*(y-map.info.origin.position.y)+(x-map.info.origin.position.x))/map.info.resolution)];
  return data;
}
