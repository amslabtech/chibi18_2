#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_link");
  ros::NodeHandle nh;

  ros::Rate loop_rate(100);

  tf::TransformBroadcaster broadcaster;

  while(nh.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_link", "laser"));
    loop_rate.sleep();
  }
}
