#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>


geometry_msgs::Twist twist;
void velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  twist = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "changer");
  ros::NodeHandle nh;
 
  ros::Subscriber changer_sub = nh.subscribe("/chibi18/velocity", 100, velocity_callback);

  ros::Publisher changer_pub = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    roomba_500driver_meiji::RoombaCtrl data;
    data.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    data.cntl = twist;
    changer_pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
