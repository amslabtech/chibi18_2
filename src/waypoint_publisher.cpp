#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

std::vector<geometry_msgs::PoseStamped> wp;

void set_pose(float, float, float);

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(10);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);

  set_pose(12, -0.5, 0);
  set_pose(12, 13.5, 0);
  set_pose(-22, 15, 0);
  set_pose(-22, 1, 0);
  set_pose(0, 0, 0);

  for(int i=0;i<wp.size();i++){
    ros::Duration(2).sleep();
    pose_pub.publish(wp[i]);
  }

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void set_pose(float x, float y, float yaw)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  wp.push_back(pose);
}
