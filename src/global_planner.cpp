#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

nav_msgs::OccupancyGrid map;
nav_msgs::Path global_path;


void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
}

int get_grid_data(float, float);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100);

  ros::Rate loop_rate(10);

  global_path.header.frame_id = "map";

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
