#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

boost::mutex target_mutex;
geometry_msgs::Pose2D target;
geometry_msgs::Pose2D temp_target;
boost::mutex roomba_odometry_mutex;
nav_msgs::Odometry roomba_odometry;
std_msgs::Bool wall_state;

void chibi18_target_callback(const geometry_msgs::Pose2DConstPtr& msg)
{
  boost::mutex::scoped_lock(target_mutex);
  target = *msg;
}

void roomba_odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
  boost::mutex::scoped_lock(roomba_odometry_mutex);
  roomba_odometry = *msg;
}

void chibi18_stop_callback(const std_msgs::BoolConstPtr& msg)
{
  wall_state = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temporaly_target");
  ros::NodeHandle nh;
  
  ros::Subscriber target_sub = nh.subscribe("/chibi18/target0", 100, chibi18_target_callback);
  
  ros::Subscriber odometry_sub = nh.subscribe("/roomba/odometry", 100, roomba_odometry_callback);
   
  ros::Subscriber stop_sub = nh.subscribe("/chibi18/stop", 100, chibi18_stop_callback);
  
  ros::Publisher target_pub =nh.advertise<geometry_msgs::Pose2D>("/chibi18/target",100);

  ros::Rate loop_rate(10);
  while(ros::ok()){
    if(wall_state.data) target_pub.publish(target);
    else{
      temp_target.x=roomba_odometry.pose.pose.orientation.x+0.5;
      temp_target.y=roomba_odometry.pose.pose.orientation.y+0.5;
      target_pub.publish(temp_target);
    }
  ros::spinOnce();
  loop_rate.sleep();
  }
}
