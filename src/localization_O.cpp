//localization_O.cpp

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_broadcaster.h"

nav_msgs::OccupancyGrid map;
sensor_msgs::LaserScan laser;
nav_msgs::Odometry odometry;
nav_msgs::Odometry pre_odometry;
geometry_msgs::PoseStamped localization;

struct Particle{
	float x;
	float y;
	int theta;
	float w;
};

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	map = *msg;
	std::cout << "map_callback" << std::endl;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
	laser = *msg;
	//std::cout << "laser_callback" << std::endl;
}

void GetRPY(const geometry_msgs::Quaternion &q, double &roll,double &pitch, double &yaw)
{
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void GetQuaternionMsg(double roll,double pitch,double yaw, geometry_msgs::Quaternion &q)
{
	tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
	quaternionTFToMsg(quat,q);
}

void odometry_callback(const nav_msgs::OdometryConstPtr &msg)
{
	pre_odometry = odometry;
	odometry = *msg;
	//std::cout << "odometry_callback" << std::endl;
}

int get_index(int x, int y)
{
	return y*map.info.width+x;
}

bool whether_inside(int x, int y)
{
	if(x<0||x>=map.info.width||y<0||y>=map.info.height)	return false;
	else	return true;
}

float gaussian(float x, float sigma2, float mean)
{
	return 1.0/sqrt(2.0*3.142*sigma2)*exp(-0.5/sigma2*(x-mean)*(x-mean));
}

float obstacle_distance_onmap(Particle p, int angle)
{
	int x0 = p.x;
	int y0 = p.y;
	int abs_ang = (angle+p.theta+270)%360;
	int count = 0;
	while(ros::ok()){
		count++;
		x0 = p.x+count*cos(abs_ang/180.0*3.142);
		y0 = p.y+count*sin(abs_ang/180.0*3.142);
		if(map.data[get_index(x0, y0)] != 0)	return count*0.05;
		if(!whether_inside(x0, y0))	return 0.0;
	}
}

float measurement_prob(Particle p)
{
	float prob = 1.0;
	const float sigma2 = 100.0;
	if(!whether_inside(p.x, p.y))	return 0.0;
	if(map.data[get_index(p.x, p.y)]!=0)	return 0.0;
	
	const int ang_resol = 6;	//[deg]
	for(int i=0;i<=720;i+=4*ang_resol){
		float dist_m = obstacle_distance_onmap(p, i/4);
		float dist_l = laser.ranges[i];
		prob *= gaussian(dist_m, sigma2, dist_l);
	}
	return prob;
}

float find_maxw(std::vector<Particle> particles, int n)
{
	float mw = particles[0].w;
	for(int i=0;i<n;i++){
		if(mw<particles[i].w)	mw = particles[i].w;
	}
	return mw;
}

void particle_filters(std::vector<Particle>& particles, int N)
{	
	std::random_device rnd;
	std::mt19937 mt(rnd());

	//---Initialization---//
	if(particles.empty()){	
		std::cout << "Initialized" << std::endl;
		while(particles.size()<=N){
			Particle particle0;
			particle0.x = rnd()%map.info.width;
			particle0.y = rnd()%map.info.height;
			particle0.theta = rnd()%360;
			if(map.data[get_index(particle0.x, particle0.y)] == 0)	particles.push_back(particle0);
		}

//		for(int i=0;i<N;i++)	std::cout << "(" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ")" << std::endl;
//		std::cout << "---------------" << std::endl;
	}

	//---Move---//
	const float straight_sigma2 = 1.0;
	const float turn_sigma2 = 1.0;
	std::normal_distribution<> straight_norm(0.0, straight_sigma2);
	std::normal_distribution<> turn_norm(0.0, turn_sigma2);

	for(int i=0;i<N;i++){
		particles[i].x += straight_norm(mt)+(odometry.pose.pose.position.x-pre_odometry.pose.pose.position.x)/0.05;
		particles[i].y += straight_norm(mt)+(odometry.pose.pose.position.y-pre_odometry.pose.pose.position.y)/0.05;
		double pre_rpy[3];
		double cur_rpy[3];
		GetRPY(pre_odometry.pose.pose.orientation, pre_rpy[0], pre_rpy[1], pre_rpy[2]);
		GetRPY(odometry.pose.pose.orientation, cur_rpy[0], cur_rpy[1], cur_rpy[2]);
		particles[i].theta += turn_norm(mt)+(cur_rpy[2]-pre_rpy[2])/3.14*180+360;
		particles[i].theta %= 360;
		particles[i].w = measurement_prob(particles[i]);
	}

//	for(int i=0;i<N;i++)	std::cout << "(" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ", " << particles[i].w << ")" << std::endl;

	//---Resampling---//
	std::uniform_real_distribution<> rand_0_1(0.0, 1.0);

	std::vector<Particle> p0;
	int index = rnd()%N;
	float beta = 0.0;
	const float mw = find_maxw(particles, N);
	for(int i=0;i<N;i++){
		beta += rand_0_1(mt)*2.0*mw;
		while(beta>particles[index].w){
			beta -= particles[index].w;
			index = (index+1)%N;
		}
		p0.push_back(particles[index]);
	}
	particles = p0;
	
	//---Localization---//
	int sum_x = 0;
	int sum_y = 0;
	int sum_theta = 0;
	for(int i=0;i<N;i++){
		sum_x += particles[i].x;
		sum_y += particles[i].y;
		sum_theta += particles[i].theta;
	}
	localization.pose.position.x = sum_x/N*map.info.resolution;
	localization.pose.position.y = sum_y/N*map.info.resolution;
	localization.pose.position.z = 0;
	GetQuaternionMsg(0.0, 0.0, (sum_theta/N-360)/180.0*3.142, localization.pose.orientation);

//	std::cout << "[" << localization.pose.position.x << "," << localization.pose.position.y << "]" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localization");
	ros::NodeHandle nh;

	laser.header.frame_id = "base_link";
	localization.header.frame_id = "map";

	ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);
	ros::Subscriber laser_sub = nh.subscribe("/base_scan", 100, laser_callback);
	ros::Subscriber odom_sub = nh.subscribe("/roomba/odometry", 100, odometry_callback);
	ros::Publisher locali_pub = nh.advertise<geometry_msgs::PoseStamped>("/chibi18/localization", 100);
	ros::Publisher array_pub = nh.advertise<geometry_msgs::PoseArray>("/chibi18/array", 100);

	std::vector<Particle> particles;
	const int N = 1000;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		if(!map.data.empty()&&!laser.ranges.empty()){
			particle_filters(particles, N);
			locali_pub.publish(localization);
			
			geometry_msgs::Pose pose;
			geometry_msgs::PoseArray array;
			array.header.frame_id = "map";
			for(int i=0;i<N;i++){
				pose.position.x = particles[i].x*map.info.resolution;
				pose.position.y = particles[i].y*map.info.resolution;
				GetQuaternionMsg(0.0, 0.0, (particles[i].theta-360)/180.0*3.142, pose.orientation);
				array.poses.push_back(pose);
			}
			array_pub.publish(array);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
