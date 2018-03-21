#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
bool pose_subscribed = false;

void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  amcl_pose = *msg;
  estimated_pose.header.frame_id = "map";
  estimated_pose.pose = amcl_pose.pose.pose;
  pose_subscribed = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl_to_chibi");
  ros::NodeHandle nh;

  ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 100, pose_callback);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/chibi18/estimated_pose", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(pose_subscribed){
      pose_pub.publish(estimated_pose);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
