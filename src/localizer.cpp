#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>

#include <random>

class Particle
{
public:
  Particle(void);

  void initialize(int, int, float, geometry_msgs::Pose, int);
  void move(float, float, float);//"odom"から見た"base_link"の動き

  geometry_msgs::PoseStamped pose;
  float likelihood;

private:

};

int N;//number of partcles
float POSITION_SIGMA;
float ORIENTATION_SIGMA;
float init_x_cov;
float init_y_cov;
float init_yaw_cov;
float init_x;
float init_y;
float init_yaw;
float odom_yaw_noise;
float odom_x_noise;
float odom_y_noise;
float range_max;
int matching_step;
float update_distance;
float update_angle;
nav_msgs::OccupancyGrid map;
bool map_subscribed = false;
std::vector<Particle>  particles;
geometry_msgs::PoseArray poses;
tf::StampedTransform current_base_link_pose;
tf::StampedTransform previous_base_link_pose;
sensor_msgs::LaserScan laser_data_from_scan;
sensor_msgs::PointCloud2 data_from_scan;
laser_geometry::LaserProjection projector;
geometry_msgs::PoseStamped estimated_pose;
bool calculate_flag = true;
float distance_sum = 0;
float angle_sum = 0;

//パーティクル配置用
std::random_device rnd;
std::mt19937 mt(rnd()); 

float get_yaw(geometry_msgs::Quaternion);
int get_grid_data(float, float);
int get_index(float, float);
float get_square(float);
bool map_valid(int, int);
float get_range_from_map(int, float, float, float);

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data_from_scan = *msg;
  for(int i=0;i<720;i++){
    if(std::isinf(laser_data_from_scan.ranges[i])){
      laser_data_from_scan.ranges[i] = range_max;
    }else if(laser_data_from_scan.ranges[i] > range_max){
      laser_data_from_scan.ranges[i] = range_max;
    }
  }
  projector.projectLaser(laser_data_from_scan, data_from_scan); 
}

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  for(int i=0;i<N;i++){
    Particle p;
    do{
      p.initialize(map.info.width, map.info.height, map.info.resolution, map.info.origin, N);
    }while(get_grid_data(p.pose.pose.position.x, p.pose.pose.position.y) != 0);
    particles.push_back(p);
    poses.poses.push_back(p.pose.pose); 
  } 
  poses.header.frame_id = "map";
  
  
  map_subscribed = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    local_nh.getParam("N", N);
    local_nh.getParam("POSITION_SIGMA", POSITION_SIGMA);
    local_nh.getParam("ORIENTATION_SIGMA", ORIENTATION_SIGMA);
    local_nh.getParam("INIT_X_COVARIANCE", init_x_cov);
    local_nh.getParam("INIT_Y_COVARIANCE", init_y_cov);
    local_nh.getParam("INIT_YAW_COVARIANCE", init_yaw_cov);
    local_nh.getParam("INIT_X", init_x);
    local_nh.getParam("INIT_Y", init_y);
    local_nh.getParam("INIT_YAW", init_yaw);
    local_nh.getParam("ODOM_X_NOISE", odom_x_noise);
    local_nh.getParam("ODOM_Y_NOISE", odom_y_noise);
    local_nh.getParam("ODOM_YAW_NOISE", odom_yaw_noise);
    local_nh.getParam("RANGE_MAX", range_max);
    local_nh.getParam("MATCHING_STEP", matching_step);
    local_nh.getParam("UPDATE_DISTANCE", update_distance);
    local_nh.getParam("UPDATE_ANGLE", update_angle);

    std::srand(time(NULL));

    ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

    ros::Publisher poses_pub= nh.advertise<geometry_msgs::PoseArray>("/chibi18/poses", 100);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/chibi18/estimated_pose", 100);

    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/chibi18/debug/scan", 100);

    tf::TransformBroadcaster map_broadcaster;
    tf::TransformListener listener;

    ros::Rate loop_rate(10);

    while(ros::ok()){
      ros::Time begin = ros::Time::now();
      if(map_subscribed && !laser_data_from_scan.ranges.empty()){
        //pridiction
        float dx = 0.0;
        float dy = 0.0;
        float dtheta = 0.0;
        geometry_msgs::Quaternion qc, qp;
        try{
          std::cout << "lookup transform odom to base_link" << std::endl;
          listener.lookupTransform("odom", "base_link", ros::Time(0), current_base_link_pose);
          dx = current_base_link_pose.getOrigin().x() - previous_base_link_pose.getOrigin().x();
          dy = current_base_link_pose.getOrigin().y() - previous_base_link_pose.getOrigin().y();
          float yaw = getYaw(previous_base_link_pose.getRotation());
          dx = dx * cos(yaw) + dy * sin(yaw);
          dy = dx * -sin(yaw) + dy * cos(yaw);
          dy = 0;//test
          std::cout << dx << ", " << dy << ", " << yaw << std::endl;
          //std::cout << "calc quat" << std::endl;
          quaternionTFToMsg(current_base_link_pose.getRotation(), qc);
          quaternionTFToMsg(previous_base_link_pose.getRotation(), qp);
          dtheta = get_yaw(qc) - get_yaw(qp);
          previous_base_link_pose = current_base_link_pose; 
          //std::cout << "calc end" << std::endl;
        }catch(tf::TransformException &ex){
          std::cout << ex.what() << std::endl;
          //continue;
        }
        distance_sum += fabs(dx);
        angle_sum += fabs(dtheta);
        if(distance_sum > update_distance){
          distance_sum = 0;
          calculate_flag = true;
        }else if(angle_sum > update_angle){
          angle_sum = 0;
          calculate_flag = true;
        }
        if(calculate_flag){
          calculate_flag = false;
          for(int i=0;i<particles.size();i++){
            particles[i].move(dx, dy, dtheta);
          }
        
          //measurement & likelihood
          std::cout << "calculate likelihood" << std::endl;
          sensor_msgs::LaserScan laser_data_from_map;
          laser_data_from_map = laser_data_from_scan; 
          laser_data_from_map.header.frame_id = "map";
          for(int i=0;i<N;i++){
            float p_yaw = get_yaw(particles[i].pose.pose.orientation);
            for(int angle=0;angle<720;angle+=matching_step){
              laser_data_from_map.ranges[angle] = get_range_from_map(angle, particles[i].pose.pose.position.x, particles[i].pose.pose.position.y, p_yaw);
            }
            //laser_pub.publish(laser_data_from_map);
            float rss = 0;//残差平方和
            for(int angle=0;angle<720;angle+=matching_step){
              rss += get_square(laser_data_from_map.ranges[angle] - laser_data_from_scan.ranges[angle]); 
            }
            particles[i].likelihood =  exp(-get_square(rss / POSITION_SIGMA) / 2.0);
            if((int)get_grid_data(particles[i].pose.pose.position.x, particles[i].pose.pose.position.y) != 0){
              //particles[i].likelihood = 0;
            }
            //std::cout << rss << ", " << particles[i].likelihood << std::endl;
          }
          float sum = 0;
          for(int i=0;i<N;i++){
            sum += particles[i].likelihood;
          }
          for(int i=0;i<N;i++){
            particles[i].likelihood /= sum;
          }

          //resampling
          std::cout << "resampling" << std::endl;
          std::vector<Particle> new_particles;
          int max_index = 0;
          for(int i=0;i<N;i++){
            if(particles[i].likelihood > particles[max_index].likelihood){
              max_index = i;
            }
          }
          float beta = 0;
          int index = rand() % N;
          for(int i=0;i<N;i++){
            beta += (rand() % N) / (float)N * 2 * particles[max_index].likelihood;
            while(beta > particles[index].likelihood){
              beta -= particles[index].likelihood;
              index = (1 + index) % N;
            }
            new_particles.push_back(particles[index]);
          }
          particles = new_particles;
        
          for(int i=0;i<N;i++){
            poses.poses[i] = particles[i].pose.pose;
          }
          poses_pub.publish(poses);
        
          //推定値の算出
          std::cout << "calculate estimated_pose" << std::endl;
          estimated_pose.header.frame_id = "map";
          /*
          float sum_angle = 0;
          for(int i=0;i<N;i++){
            estimated_pose.pose.position.x += particles[i].likelihood * particles[i].pose.pose.position.x; 
            estimated_pose.pose.position.y += particles[i].likelihood * particles[i].pose.pose.position.y; 
            sum_angle += particles[i].likelihood * get_yaw(particles[i].pose.pose.orientation);
          } 
          tf::Quaternion temp_tf_q = tf::createQuaternionFromYaw(sum_angle);
          geometry_msgs::Quaternion temp_g_q;
          quaternionTFToMsg(temp_tf_q, temp_g_q);
          estimated_pose.pose.orientation = temp_g_q;
          */
          estimated_pose = particles[max_index].pose;
          pose_pub.publish(estimated_pose);
        }
        //odomの補正を計算
        std::cout << "modfy frame odom" << std::endl;
        try{
          tf::StampedTransform _transform;
          _transform.stamp_ = ros::Time::now();
          _transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x, estimated_pose.pose.position.y, 0.0));
          _transform.setRotation(tf::createQuaternionFromYaw(get_yaw(estimated_pose.pose.orientation)));
          tf::Stamped<tf::Pose> tf_stamped(_transform.inverse(), laser_data_from_scan.header.stamp, "base_link");
          tf::Stamped<tf::Pose> odom_to_map; 
          listener.transformPose("odom", tf_stamped, odom_to_map);
          tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
          tf::StampedTransform temp_tf_stamped(latest_tf.inverse(), laser_data_from_scan.header.stamp, "map", "odom");
          map_broadcaster.sendTransform(temp_tf_stamped);
        }catch(tf::TransformException ex){
          std::cout << "braodcast error!" << std::endl;
          std::cout << ex.what() << std::endl; 
        }
        std::cout << "from map to odom transform broadcasted" << std::endl;
      }
      
      std::cout << "loop:" << ros::Time::now() - begin << "[s]" << std::endl;
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}

float get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

int get_grid_data(float x, float y)
{
  int data = map.data[get_index(x, y)];
  return data;
}

int get_index(float x, float y)
{
  
  if(x > 0){
    x = ((int)(10*(2*x)+1))/20.0;
  }else{
    x = ((int)(10*(2*x)))/20.0;
  }
  if(y > 0){
    y = ((int)(10*(2*y)))/20.0;
  }else{
    y = ((int)(10*(2*y)-1))/20.0;
  }
  
  int index = int((map.info.width*(y-map.info.origin.position.y)+(x-map.info.origin.position.x))/map.info.resolution);
  //std::cout << index << " " << x << " " << y <<std::endl;
  return index;
}

Particle::Particle(void)
{
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
}

void Particle::initialize(int width, int height, float resolution, geometry_msgs::Pose origin, int N)
{
  std::normal_distribution<> rand_x(init_x, init_x_cov);
  pose.pose.position.x = rand_x(mt);
  std::normal_distribution<> rand_y(init_y, init_y_cov);
  pose.pose.position.y = rand_y(mt);
  std::normal_distribution<> rand_yaw(init_yaw, init_yaw_cov);
  quaternionTFToMsg(tf::createQuaternionFromYaw(rand_yaw(mt)), pose.pose.orientation);
  likelihood = 1.0 / (float)N;
}

void Particle::move(float dx, float dy, float dtheta)
{
  //引数はbase_linkの正面方向をxとした動き
  float yaw = get_yaw(pose.pose.orientation);
  std::normal_distribution<> rand_x(0, odom_x_noise);
  dx += rand_x(mt);
  std::normal_distribution<> rand_y(0, odom_y_noise);
  dy += rand_y(mt);
  std::normal_distribution<> rand_yaw(0, odom_yaw_noise);
  pose.pose.position.x += dx * cos(yaw) - dy * sin(yaw);
  pose.pose.position.y += dx * sin(yaw) + dy * cos(yaw);
  quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + dtheta + rand_yaw(mt)), pose.pose.orientation); 
}

float get_square(float value)
{
  return value * value;
}

bool map_valid(int i, int j)
{
  return (i>=0) && (i<map.info.width) && (j>=0) && (j<map.info.height);
}


float get_range_from_map(int angle, float ox, float oy, float yaw)
{
  int index0 = get_index(ox, oy);
  int x0 = index0 % map.info.width; 
  int y0 = (index0 - x0) / map.info.width; 
  int x = 0, y = 0;
  float _angle = angle*laser_data_from_scan.angle_increment - M_PI/2.0;
  int index1 = get_index(ox + range_max * cos(yaw + _angle), oy + range_max * sin(yaw + _angle));
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
  return range_max;
}
