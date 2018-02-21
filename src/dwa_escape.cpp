#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//物理量に修正すること
const float MAX_VELOCITY = 0.45;
const float MAX_ANGULAR_VELOCITY = 1.0;
const float MAX_ACCELERATION = 1.5;
const float MAX_ANGULAR_ACCELERATION = 20.0;
const float VELOCITY_RESOLUTION = 0.05;
const float ANGULAR_VELOCITY_RESOLUTION = 0.05;
const float INTERVAL = 0.100;

//評価関数の係数
const float ALPHA = 0.1;
const float BETA = 0.2;
const float GAMMA = 0.1;

//DynamicWindowの辺
float window_left = -MAX_ANGULAR_VELOCITY;
float window_up = MAX_VELOCITY;
float window_right = MAX_ANGULAR_VELOCITY;
float window_down = -MAX_VELOCITY;

geometry_msgs::Pose2D goal;

//subscribe用
nav_msgs::Odometry previous_odometry;
nav_msgs::Odometry current_odometry;
geometry_msgs::Twist velocity_odometry;
sensor_msgs::LaserScan laser_data;

void evaluate(geometry_msgs::Twist&);
float calcurate_heading(float, float, geometry_msgs::Point);
float calcurate_distance(geometry_msgs::Point, float);
float calcurate_velocity(float);
void calcurate_dynamic_window(void);

float get_larger(float, float);
float get_smaller(float, float);
float get_yaw(geometry_msgs::Quaternion);

void odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
  previous_odometry = current_odometry;
  current_odometry = *msg;
  velocity_odometry.linear.x = sqrt(pow(current_odometry.pose.pose.position.x-previous_odometry.pose.pose.position.x, 2) + pow(current_odometry.pose.pose.position.y-previous_odometry.pose.pose.position.y, 2))/INTERVAL;
  velocity_odometry.angular.z = (get_yaw(current_odometry.pose.pose.orientation)-get_yaw(previous_odometry.pose.pose.orientation))/INTERVAL;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_escape");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    float distance;
    local_nh.getParam("DISTANCE", distance);

    ros::Subscriber odometry_sub = nh.subscribe("/roomba/odometry", 100, odometry_callback);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/chibi18/velocity", 100);

    //ゴールの座標設定
    goal.x = distance;
    goal.y = 0.0;

    ros::Rate loop_rate(10);

    geometry_msgs::Twist velocity;

    while(ros::ok()){
      evaluate(velocity);
      velocity_pub.publish(velocity);
      ros::spinOnce();
      loop_rate.sleep();
    }
}


void evaluate(geometry_msgs::Twist& velocity)
{
  std::vector<std::vector<float> > e;
  e.resize(int(window_up/VELOCITY_RESOLUTION));
  for(int i = 0;i<int(window_up/VELOCITY_RESOLUTION);i++){
    e[i].resize(int(window_right - window_left)/ANGULAR_VELOCITY_RESOLUTION);
  }
  for(float v = 0;v < window_up/VELOCITY_RESOLUTION;v++){
    for(float o = 0;o < (window_right-window_left)/ANGULAR_VELOCITY_RESOLUTION;o++){
      e[v][o] = ALPHA * calcurate_heading(window_left+o*ANGULAR_VELOCITY_RESOLUTION, get_yaw(current_odometry.pose.pose.orientation), current_odometry.pose.pose.position) + BETA * calcurate_distance(current_odometry.pose.pose.position, VELOCITY_RESOLUTION*v) + GAMMA * calcurate_velocity(VELOCITY_RESOLUTION*v);
      //std::cout << e[v][o] << " ";
    }
    //std::cout << std::endl;
  }
  int j = 0;
  int k = 0;
  float max = 0;
  for(float v = 0;v < window_up/VELOCITY_RESOLUTION;v++){
    for(float o = 0;o < (window_right-window_left)/ANGULAR_VELOCITY_RESOLUTION;o++){
      if(e[v][o] > max){
        max = e[v][o]; 
	j=v;
	k=o;
      }
    }
  }
  velocity.linear.x = j * VELOCITY_RESOLUTION / MAX_VELOCITY * 0.5;
  if(sqrt(pow(goal.x - current_odometry.pose.pose.position.x, 2) + pow(goal.y-current_odometry.pose.pose.position.y, 2)) < 0.5){
    velocity.linear.x *= sqrt(pow(goal.x - current_odometry.pose.pose.position.x, 2) + pow(goal.y-current_odometry.pose.pose.position.y, 2));
  }
  velocity.angular.z = (window_left + k * ANGULAR_VELOCITY_RESOLUTION) / MAX_ANGULAR_VELOCITY * 0.5;
  if(sqrt(pow(goal.x - current_odometry.pose.pose.position.x, 2) + pow(goal.y-current_odometry.pose.pose.position.y, 2)) < 0.01){
    velocity.angular.z *= sqrt(pow(goal.x - current_odometry.pose.pose.position.x, 2) + pow(goal.y-current_odometry.pose.pose.position.y, 2));
  }
  //std::cout << velocity_odometry << std::endl;
  std::cout << current_odometry.pose.pose << std::endl;
  std::cout << velocity.linear.x << " " << velocity.angular.z << std::endl;
  //std::cout << window_left << " " << ANGULAR_VELOCITY_RESOLUTION << std::endl;
  //std::cout << velocity.angular.z << std::endl;
  std::cout << j << std::endl;
  std::cout << k << std::endl;
  std::cout << "max:" << max << std::endl;
  std::cout << std::endl;
}

float calcurate_heading(float omega, float angle, geometry_msgs::Point point)
{
  angle = 0;//TEST DATA
  angle = get_yaw(current_odometry.pose.pose.orientation);
  float val = 180 - fabs(atan2((goal.y-point.y), (goal.x-point.x)) - (angle + omega * INTERVAL)) / M_PI * 180;
  return val;
}

float calcurate_distance(geometry_msgs::Point point, float v)
{
  float min_distance = 60;
  if(!laser_data.ranges.empty()){
    for(int i=0;i<720;i++){
      float distance = laser_data.ranges[i] - v * INTERVAL;
      if(min_distance > distance){
        min_distance = distance;
      }
    }
  }
  return min_distance;
}

float calcurate_velocity(float v)
{
  return v;
}

void calcuate_dynamic_window(void)
{
  window_left = get_larger(-MAX_ANGULAR_VELOCITY, velocity_odometry.angular.z-MAX_ANGULAR_ACCELERATION*INTERVAL);
  window_up = get_smaller(MAX_VELOCITY, velocity_odometry.linear.x+MAX_ACCELERATION*INTERVAL);
  window_right = get_smaller(MAX_ANGULAR_VELOCITY, velocity_odometry.angular.z+MAX_ANGULAR_ACCELERATION*INTERVAL);
  window_down = get_larger(0, velocity_odometry.linear.x-MAX_ACCELERATION*INTERVAL);
}

float get_larger(float a, float b)
{
  if(a > b){
    return a;
  }else{
    return b;
  }
}

float get_smaller(float a, float b)
{
  if(a < b){
    return a;
  }else{
    return b;
  }
}

float get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}
