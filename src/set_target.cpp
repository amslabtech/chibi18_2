#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PointStamped.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_target");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  geometry_msgs::PointStamped target;
  local_nh.getParam("X", target.point.x);
  local_nh.getParam("Y", target.point.y);
  target.header.frame_id = "odom";

  std::cout << "x=" << target.point.x << " y=" << target.point.y << std::endl;

  ros::Publisher target_pub = nh.advertise<geometry_msgs::PointStamped>("/chibi18/target", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    target_pub.publish(target);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
