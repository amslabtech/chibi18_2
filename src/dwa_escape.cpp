#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <std_msgs/Bool.h>

//物理量に修正すること
const float MAX_VELOCITY = 0.45;
const float MAX_ANGULAR_VELOCITY = 2.0;
const float MAX_ACCELERATION = 1.5;
const float MAX_ANGULAR_ACCELERATION = 6.0;
const float VELOCITY_RESOLUTION = 0.01;
const float ANGULAR_VELOCITY_RESOLUTION = 0.05;
const float INTERVAL = 0.100;
const float SIMULATE_TIME = 2.500;
const float LASER_RESOLUTION = 0.00436332312;//[rad]

//評価関数の係数
const float ALPHA = 0.10;//0.20;//heading
const float BETA = 1.0;//distance
const float GAMMA = 1.8;//1.00;//velocity

//DynamicWindowの辺
float window_left = -MAX_ANGULAR_VELOCITY;
float window_up = MAX_VELOCITY;
float window_right = MAX_ANGULAR_VELOCITY;
float window_down = -MAX_VELOCITY;

//subscribe用
nav_msgs::Odometry previous_odometry;
nav_msgs::Odometry current_odometry;
geometry_msgs::Twist velocity_odometry;
geometry_msgs::Pose2D goal;
sensor_msgs::LaserScan laser_data;
sensor_msgs::LaserScan _laser_data;//計算用
bool odometry_subscribed = false;
bool target_subscribed = false;
bool move_allowed = true;

void evaluate(geometry_msgs::Twist&);
float calcurate_heading(float, float, geometry_msgs::Point);
float calcurate_distance(float, float);
float calcurate_velocity(float);
void calcurate_dynamic_window(void);

float get_larger(float, float);
float get_smaller(float, float);
float get_yaw(geometry_msgs::Quaternion);

void target_callback(const geometry_msgs::Pose2DConstPtr& msg)
{
  goal = *msg;
  target_subscribed = true;
}

void odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
  previous_odometry = current_odometry;
  current_odometry = *msg;
  velocity_odometry.linear.x = sqrt(pow(current_odometry.pose.pose.position.x-previous_odometry.pose.pose.position.x, 2) + pow(current_odometry.pose.pose.position.y-previous_odometry.pose.pose.position.y, 2))/INTERVAL;
  velocity_odometry.angular.z = (get_yaw(current_odometry.pose.pose.orientation)-get_yaw(previous_odometry.pose.pose.orientation))/INTERVAL;
  if(!isnan(velocity_odometry.angular.z)){
    odometry_subscribed = true;
  }
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data = *msg;
}

void stopper_callback(const std_msgs::BoolConstPtr& msg)
{
  move_allowed = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_escape");
    ros::NodeHandle nh;

    ros::Subscriber odometry_sub = nh.subscribe("/roomba/odometry", 100, odometry_callback);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/chibi18/velocity", 100);

    ros::Subscriber target_sub = nh.subscribe("/chibi18/target", 100, target_callback);

    ros::Subscriber stopper_sub = nh.subscribe("/chibi18/stop", 100, stopper_callback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist velocity;

    while(ros::ok()){
      if(odometry_subscribed && target_subscribed && !laser_data.ranges.empty()){
        calcurate_dynamic_window();
        evaluate(velocity);
        float ratio = 0.25;
        float distance_to_goal = sqrt(pow(goal.x - current_odometry.pose.pose.position.x, 2) + pow(goal.y-current_odometry.pose.pose.position.y, 2));
        if(distance_to_goal < 0.5){
          velocity.linear.x *= 2*distance_to_goal;
        }
        if(distance_to_goal < 0.05){
          velocity.angular.z *= distance_to_goal;
        }
        velocity.linear.x *= ratio;
        velocity.angular.z *= (1-ratio);

        //stopの時
        if(!move_allowed){
          velocity.linear.x = 0;
        }

        std::cout << velocity << std::endl;

        velocity_pub.publish(velocity);
        std::cout << "goal:" <<  goal.x <<" "<< goal.y << std::endl;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
}


void evaluate(geometry_msgs::Twist& velocity)
{
  std::vector<std::vector<float> > e;
  int elements_v = int((window_up - window_down)/VELOCITY_RESOLUTION);
  int elements_o = int((window_right - window_left)/ANGULAR_VELOCITY_RESOLUTION);
  //std::cout << "left=" << window_left << " right=" << window_right << std::endl;
  //std::cout << "v=" << elements_v << " o=" << elements_o << std::endl;
  //std::cout << velocity_odometry << std::endl;
  e.resize(elements_v);
  for(int i = 0;i<elements_v;i++){
    e[i].resize(elements_o);
  }
  _laser_data = laser_data;
  for(float v = 0;v < elements_v;v++){
    for(float o = 0;o < elements_o;o++){
      float _velocity = window_down + v * VELOCITY_RESOLUTION;
      float _omega = window_left + o * ANGULAR_VELOCITY_RESOLUTION; 
      e[v][o] = ALPHA * calcurate_heading(_omega, get_yaw(current_odometry.pose.pose.orientation), current_odometry.pose.pose.position) + BETA * calcurate_distance(_velocity, _omega) + GAMMA * calcurate_velocity(_velocity);
      std::cout << e[v][o] << " ";
    }
    std::cout << std::endl;
  }
  int j = 0;
  int k = 0;
  float max = 0;
  for(float v = 0;v < elements_v;v++){
    for(float o = 0;o < elements_o;o++){
      if(e[v][o] > max){
        max = e[v][o]; 
	j=v;
	k=o;
      }
    }
  }
  velocity.linear.x = (window_down + j * VELOCITY_RESOLUTION) / MAX_VELOCITY;
  velocity.angular.z = (window_left + k * ANGULAR_VELOCITY_RESOLUTION) / MAX_ANGULAR_VELOCITY;
  std::cout << current_odometry.pose.pose << std::endl;
  //std::cout << velocity.linear.x << " " << velocity.angular.z << std::endl;
  //std::cout << window_left << " " << ANGULAR_VELOCITY_RESOLUTION << std::endl;
  //std::cout << velocity.angular.z << std::endl;
  //std::cout << j << std::endl;
  //std::cout << k << std::endl;
  //std::cout << "max:" << max << std::endl;
  std::cout << std::endl;
}

float calcurate_heading(float omega, float angle, geometry_msgs::Point point)
{
  float goal_angle = (atan2((goal.y-point.y), (goal.x-point.x)) - (angle + omega * INTERVAL));// / M_PI * 180;
  //std::cout << atan2(sin(goal_angle), cos(goal_angle)) / M_PI * 180<< "[deg]" << std::endl;
  float val = 180 - fabs(atan2(sin(goal_angle), cos(goal_angle))) / M_PI * 180;
  //std::cout << val << " ";
  return val;
}

float calcurate_distance(float v, float omega)
{
  geometry_msgs::Pose2D position;
  /*
  if(omega != 0.0){
    position.x = v / omega * sin(omega * SIMULATE_TIME);
    position.y = v / omega * cos(omega * SIMULATE_TIME);
  }else{
    position.x = v * SIMULATE_TIME;
    position.y = 0;
  }
  */
  float dt = 0.01;
  for(float t=0;t<SIMULATE_TIME;t+=dt){
    position.x += v * cos(omega * t) * dt;
    position.y += v * sin(omega * t) * dt;
  }

  //std::cout << std::setprecision(2) << "(" << v << ", " << omega << ")";
  //std::cout << std::setprecision(2) << "(" << position.x << ", " << position.y << ")";

  geometry_msgs::Pose2D object;

  int index = 0;

  const float LIMIT_DISTANCE = 2.0;
  float distance = LIMIT_DISTANCE;
  for(int i=60;i<660;i++){//15~165
    if(_laser_data.ranges[i] < LIMIT_DISTANCE){
      object.x = _laser_data.ranges[i] * sin(LASER_RESOLUTION * i);
      object.y = _laser_data.ranges[i] * cos(LASER_RESOLUTION * i) * -1.0;
      /*
      if((object.y < position.y) && (omega > 0) && (object.y > 0)){
        //std::cout << "b";
        return 0;
      }else if((object.y > position.y) && (omega < 0) && (object.y < 0)){
        //std::cout << "c";
        return 0;
      }else if(object.x < position.x){
        //std::cout << "a";
        return 0;
      }
      */
      //std::cout << object << " ";
      float _distance = sqrt(pow((object.x - position.x), 2) + pow((object.y - position.y), 2)); 
      if(_distance < distance){
        //std::cout << "obj" << object << "pos" << position << std::endl;
        index = i;
        distance = _distance;
      }
      if(distance < 0.3){
        return 0;
      }
    }
  }
  //std::cout << "(" << _laser_data.ranges[index] * sin(LASER_RESOLUTION * index)  << "," << _laser_data.ranges[index] * cos(LASER_RESOLUTION * index) * -1 << ") ";
  //std::cout << v << "[m/s]" << omega << "[rad/s]" << std::endl;
  //std::cout << position << " ";
  //std::cout << distance << " ";
  return distance;
}

float calcurate_velocity(float v)
{
  //std::cout << v << " ";
  return v;
}

void calcurate_dynamic_window(void)
{
  window_left = get_larger(-MAX_ANGULAR_VELOCITY, velocity_odometry.angular.z-MAX_ANGULAR_ACCELERATION*INTERVAL);
  window_up = get_smaller(MAX_VELOCITY, velocity_odometry.linear.x+MAX_ACCELERATION*INTERVAL);
  window_right = get_smaller(MAX_ANGULAR_VELOCITY, velocity_odometry.angular.z+MAX_ANGULAR_ACCELERATION*INTERVAL);
  window_down = get_larger(0, velocity_odometry.linear.x-MAX_ACCELERATION*INTERVAL);

  if(window_left > window_right){
    window_left = -MAX_ANGULAR_VELOCITY;
    window_right = MAX_ANGULAR_VELOCITY;
  }
  if(window_down > window_up){
    window_up = MAX_VELOCITY;
    window_down = 0;
  }
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
