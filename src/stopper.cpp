#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>

sensor_msgs::LaserScan data;


void laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  data = *msg;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stopper");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

  ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/chibi18/stop", 100);


  ros::Rate loop_rate(10);
  
  while(ros::ok()){
    //std::cout << data << std::endl;
    std_msgs::Bool state;
    if(!data.ranges.empty()){
      std::cout << data << std::endl;
      if(data.ranges[360] < 0.5){
        state.data = false;
      }else{
        state.data = true;
      }
      stop_pub.publish(state);
    }else{
      state.data = true;
      stop_pub.publish(state);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
