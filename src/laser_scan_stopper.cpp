#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

class ScanStopper {
	public:
		//可調整パラメータ類
		const static double FORWARD_SPEED_MPS = 0.5;
		const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
		const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
		//最小レンジ
		const static float MIN_PROXIMITY_RANGE_M = 0.1;
		ScanStopper();
	private:
		ros::NodeHandle nh;
		ros::Publisher cmd_pub;
		ros::Subscriber laser_sub;
		bool keepMoving;
		
		void moveForward();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

ScanStopper::ScanStopper()
{
	keepMoving = true;
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	laser_sub = nh.subscribe("base_scan",1,&ScanStopper::scanCallback,this);
}

void ScanStopper::moveForward(){
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	cmd_pub.publish(msg);
}

void ScanStopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	init minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min)/scan->angle_increment);
	init maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min)/scan->angle_increment);
	
	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex;currIndex++){
		if (scan->ranges[currINdex] < closestRange){
			closestRange = scan->ranges[currIndex];
		}
	}
	ROS_INFO_STREAM("closest range: " << closestRange);
	
	if (closestRange < MIN_PROXIMITY_RANGE_M){
		ROS_INFO("Stop!");
		keepMoving = false;
	}
}

void ScanStopper::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");
	
	while(ros::ok() && keepMoving){
		moveForward();
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"stopper");
	ScanStopper stopper;
	stopper.startMoving();
	return 0;
}
