#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>

boost::mutex laser_mutex;
sensor_msgs::LaserScan data;


void laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  boost::mutex::scoped_lock(laser_mutex);
  data = *msg;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "stopper");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

  ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/chibi18/stop", 100);


  ros::Rate loop_rate(10);
  sensor_msgs::LaserScan _data;
  
  while(ros::ok()){
    {
      boost::mutex::scoped_lock(laser_mutex);
      _data = data;
    } 
    if(!_data.ranges.empty()){
      for(int i=0;i<(sizeof(_data.ranges)/sizeof(_data.ranges[0]));i++){
        std::cout << _data.ranges[i] << std::endl;
      }
      std::cout << std::endl;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
