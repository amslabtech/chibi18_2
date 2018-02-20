#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::LaserScan data;

void laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  data = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "runner");
  ros::NodeHandle nh;
 
  ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/chibi18/velocity", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    geometry_msgs::Twist velocity;
    if(!data.ranges.empty()){
      int index = 0;
      float min_range = 60;
      for(int i=240;i<480;i++){
        if(min_range > data.ranges[i]){
          index = i;
          min_range = data.ranges[i];
        }
      } 
      std::cout << min_range << "[m]" << std::endl;
      if(min_range < 0.8){
        if(index<360){
          velocity.linear.x = 0;
          velocity.angular.z = 0.5;
        }else{
          velocity.linear.x = 0;
          velocity.angular.z = -0.5;
        }
      }else{
        velocity.linear.x = 0.5;
        velocity.angular.z = 0;
      }
      velocity_pub.publish(velocity);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
