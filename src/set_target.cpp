#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_target");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  float distance;
  local_nh.getParam("DISTANCE", distance);

  ros::Publisher target_pub = nh.advertise<geometry_msgs::Pose2D>("/chibi18/target", 100);

  ros::Rate loop_rate(10);

  geometry_msgs::Pose2D target;
  target.x = distance;
  target.y = 0.0;
  target.theta = 0.0;
  while(ros::ok()){
    target_pub.publish(target);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
