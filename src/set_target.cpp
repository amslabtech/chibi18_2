#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_target");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  geometry_msgs::Pose2D target;
  local_nh.getParam("X", target.x);
  local_nh.getParam("Y", target.y);
  target.theta = 0.0;
  std::cout << "x=" << target.x << " y=" << target.y << std::endl;

  ros::Publisher target_pub = nh.advertise<geometry_msgs::Pose2D>("/chibi18/target", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    target_pub.publish(target);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
