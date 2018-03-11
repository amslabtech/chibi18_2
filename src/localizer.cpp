#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    while(1){

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
