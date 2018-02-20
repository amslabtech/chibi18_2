#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "runner");
  ros::NodeHandle nh;
 
  ros::Rate loop_rate(10);
  while(ros::ok()){



    ros::spinOnce();
    loop_rate.sleep();
  }
}
