#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

tf::StampedTransform odom_to_base_link;
geometry_msgs::Twist velocity;
sensor_msgs::LaserScan laser;
geometry_msgs::Pose pose;
nav_msgs::OccupancyGrid map;

bool map_subscribed = false;

const double DT = 0.1;//[s]
const double RANGE_MAX = 15;

double get_yaw(geometry_msgs::Quaternion);
double get_range_from_map(int, float, float, float);
bool map_valid(int, int);
int get_grid_data(float, float);
int get_index(float, float);
int get_i_from_x(float);
int get_j_from_y(float);

void velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
  velocity = *msg;
}

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  map_subscribed = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roomba_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Subscriber velocity_sub = nh.subscribe("/chibi18/velocity", 100, velocity_callback);

  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 100);

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

  tf::TransformBroadcaster odom_broadcaster;

  laser.header.frame_id = "laser";
  laser.angle_min = -M_PI / 2.0;
  laser.angle_max = M_PI / 2.0;
  laser.angle_increment = M_PI / 720.0;
  laser.scan_time = 0.025;
  laser.time_increment = laser.scan_time / (720.0 * 2.0);
  laser.range_min = 0.40;
  laser.range_max = 60.0;
  laser.ranges.resize(720);

  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  odom_to_base_link = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), "odom", "base_link");

  ros::Rate loop_rate(10);

  while(ros::ok()){
    double distance = velocity.linear.x * DT;
    pose.position.x += distance * cos(get_yaw(pose.orientation));
    pose.position.y += distance * sin(get_yaw(pose.orientation));
    pose.orientation = tf::createQuaternionMsgFromYaw(get_yaw(pose.orientation) + velocity.angular.z * DT);

    if(map_subscribed){
      double yaw = get_yaw(pose.orientation);
      for(int angle=0;angle<720;angle++){
        laser.ranges[angle] = get_range_from_map(angle, pose.position.x, pose.position.y, yaw);
      }
      laser.header.stamp = ros::Time::now();
      laser_pub.publish(laser);
    }
    try{
      odom_to_base_link = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(get_yaw(pose.orientation)), tf::Vector3(pose.position.x, pose.position.y, 0.0)), ros::Time::now(), "odom", "base_link");
      odom_broadcaster.sendTransform(odom_to_base_link);
    }catch(tf::TransformException ex){
      std::cout << "broadcast error" << std::endl;
      std::cout << ex.what() << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

double get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

double get_range_from_map(int angle, float ox, float oy, float yaw)
{
  int index0 = get_index(ox, oy);
  int x0 = index0 % map.info.width;
  int y0 = (index0 - x0) / map.info.width;
  int x = 0, y = 0;
  float _angle = angle*laser.angle_increment - M_PI/2.0;
  int index1 = get_index(ox + RANGE_MAX * cos(yaw + _angle), oy + RANGE_MAX * sin(yaw + _angle));
  int x1 = index1 % map.info.width;
  int y1 = (index1 - x1) / map.info.width;
  int steep;
  int xstep, ystep;
  int temp;
  int deltax, deltay, error, deltaerr;
  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep){
    temp = x0;
    x0 = y0;
    y0 = temp;

    temp = x1;
    x1 = y1;
    y1 = temp;
  }
  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;
  x = x0;
  y = y0;
  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep){
    if(!map_valid(y, x) || ((int)(map.data[x * map.info.width + y])!=0)){
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
    }
  }else{
    if(!map_valid(x, y) || ((int)(map.data[y * map.info.width + x])!=0)){
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
    }
  }
  while(x != (x1 + xstep * 1)){
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax){
      y += ystep;
      error -= deltax;
    }
    if(steep){
      if(!map_valid(y, x) || ((int)(map.data[x * map.info.width + y])!=0)){
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
      }
    }else{
      if(!map_valid(x, y) || ((int)(map.data[y * map.info.width + x])!=0)){
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
      }
    }
  }
  return RANGE_MAX;
 }

int get_grid_data(float x, float y)
{
  int data = map.data[get_index(x, y)];
  return data;
}

int get_index(float x, float y)
{
  int index = map.info.width * get_j_from_y(y) + get_i_from_x(x);
  //std::cout << index << " " << x << " " << y <<std::endl;
  return index;
}

int get_i_from_x(float x)
{
  return floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
}

int get_j_from_y(float y)
{
  return floor((y - map.info.origin.position.y) / map.info.resolution + 0.5);
}

bool map_valid(int i, int j)
{
  return (i>=0) && (i<map.info.width) && (j>=0) && (j<map.info.height);
}
