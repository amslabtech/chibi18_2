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
float odom_xy_noise;
float range_max;
int matching_step;
nav_msgs::OccupancyGrid map;
bool map_subscribed = false;
std::vector<Particle>  particles;
geometry_msgs::PoseArray poses;
geometry_msgs::TransformStamped transform;
tf::StampedTransform current_base_link_pose;
tf::StampedTransform previous_base_link_pose;
sensor_msgs::LaserScan laser_data_from_scan;
sensor_msgs::PointCloud2 data_from_scan;
laser_geometry::LaserProjection projector;

//パーティクル配置用
std::random_device rnd;
std::mt19937 mt(rnd()); 

float get_yaw(geometry_msgs::Quaternion);
int get_grid_data(float, float);
int get_index(float, float);
void set_transform(float, float, float);

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
    local_nh.getParam("ODOM_XY_NOISE", odom_xy_noise);
    local_nh.getParam("ODOM_YAW_NOISE", odom_yaw_noise);
    local_nh.getParam("RANGE_MAX", range_max);
    local_nh.getParam("MATCHIING_STEP", matching_step);

    std::srand(time(NULL));

    ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

    ros::Publisher poses_pub= nh.advertise<geometry_msgs::PoseArray>("/chibi18/poses", 100);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/chibi18/estimated_pose", 100);

    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/chibi18/debug/scan", 100);

    tf::TransformBroadcaster map_broadcaster;
    tf::TransformListener listener;

    ros::Rate loop_rate(10);

    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";

    set_transform(init_x, init_y, init_yaw);//適当

    while(ros::ok()){
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
          std::cout << "calc quat" << std::endl;
          quaternionTFToMsg(current_base_link_pose.getRotation(), qc);
          quaternionTFToMsg(previous_base_link_pose.getRotation(), qp);
          dtheta = get_yaw(qc) - get_yaw(qp);
          previous_base_link_pose = current_base_link_pose; 
          std::cout << "calc end" << std::endl;
        }catch(tf::TransformException &ex){
          std::cout << ex.what() << std::endl;
          //continue;
        }
         
        for(int i=0;i<particles.size();i++){
          //particles[i].move(0.01, 0.01, 0.05);//適当
          particles[i].move(dx, dy, dtheta);
        }
        
        //measurement & likelihood
        
        std::cout << "calculate likelihood" << std::endl;
        sensor_msgs::LaserScan laser_data_from_map;
        laser_data_from_map = laser_data_from_scan; 
        laser_data_from_map.header.frame_id = "map";
        for(int i=0;i<N;i++){
          //std::cout << "N=" << i << std::endl;
          Eigen::Vector3d obstacle_map;//mapフレームから見たある障害物の位置
          obstacle_map(2) = 1;
          Eigen::Matrix3d laser_to_map;//laserフレームからmapフレームへの変換
          laser_to_map << cos(get_yaw(particles[i].pose.pose.orientation)), -sin(get_yaw(particles[i].pose.pose.orientation)), particles[i].pose.pose.position.x,
                          sin(get_yaw(particles[i].pose.pose.orientation)), cos(get_yaw(particles[i].pose.pose.orientation)), particles[i].pose.pose.position.y,
                          0, 0, 1;
          Eigen::Vector3d obstacle_laser;//laserフレームから見たある障害物の位置
          obstacle_laser(2) = 1;
          //std::cout << "p:" << particles[i].pose.pose.position.x << ", " << particles[i].pose.pose.position.y << ", " << get_yaw(particles[i].pose.pose.orientation) << std::endl;
          for(int angle=0;angle<720;angle++){
            laser_data_from_map.ranges[angle] = -1;
            float _angle = angle*laser_data_from_scan.angle_increment - M_PI/2.0;
            //std::cout << _angle << "[rad]" << std::endl;
            for(float distance=0;distance<range_max;distance+=map.info.resolution){
              obstacle_laser(0) = distance * cos(_angle);
              obstacle_laser(1) = distance * sin(_angle); 
              obstacle_map = laser_to_map * obstacle_laser;
              if(get_grid_data(obstacle_map(0), obstacle_map(1)) == 100){
                laser_data_from_map.ranges[angle] = sqrt(pow(obstacle_laser(0), 2) + pow(obstacle_laser(1), 2));
                int _x = get_index(obstacle_map(0), obstacle_map(1)) % map.info.width;
                float __x = _x * map.info.resolution+map.info.origin.position.x;
                int _y = (get_index(obstacle_map(0), obstacle_map(1)) - _x) / map.info.width;
                float __y = _y * map.info.resolution+map.info.origin.position.y;
                //std::cout << obstacle_map(0) << ", " << obstacle_map(1) << ", " << get_index(obstacle_map(0), obstacle_map(1)) << ", " << __x  << ", " << __y << ", " << get_index(__x, __y) << std::endl;
                break;
              }
            } 
            if(laser_data_from_map.ranges[angle] < 0){
              laser_data_from_map.ranges[angle] = range_max;
            }
            //std::cout << angle << ":" << laser_data_from_map.ranges[angle] << std::endl;
          }
          //laser_pub.publish(laser_data_from_map);
          float rss = 0;//残差平方和
          for(int angle=0;angle<720;angle++){
            rss += pow(laser_data_from_map.ranges[angle] - laser_data_from_scan.ranges[angle], 2); 
          }
          particles[i].likelihood =  exp(-pow(rss / POSITION_SIGMA, 2) / 2.0);
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
        geometry_msgs::PoseStamped estimated_pose;
        estimated_pose.header.frame_id = "map";
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
        pose_pub.publish(estimated_pose);

        //odomの補正を計算
        std::cout << "modfy frame odom" << std::endl;
        transform.header.stamp = ros::Time::now();
        /*
        transform.transform.translation.x += estimated_pose.pose.position.x - current_base_link_pose.getOrigin().x();
        transform.transform.translation.y += estimated_pose.pose.position.y - current_base_link_pose.getOrigin().y();
        float d_theta = get_yaw(estimated_pose.pose.orientation) - get_yaw(qc);
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(d_theta);
        geometry_msgs::Quaternion g_q;
        quaternionTFToMsg(tf_q, g_q);
        transform.transform.rotation.x += g_q.x;
        transform.transform.rotation.y += g_q.y;
        transform.transform.rotation.z += g_q.z;
        transform.transform.rotation.w += g_q.w;
        */
        map_broadcaster.sendTransform(transform);
        std::cout << "from map to odom transform broadcasted" << std::endl;
        
      }
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

void set_transform(float x, float y, float yaw)
{
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), transform.transform.rotation);
 
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

void Particle::move(float x, float y, float yaw)
{
  std::normal_distribution<> rand_xy(0, odom_xy_noise);
  pose.pose.position.x += x + rand_xy(mt);
  pose.pose.position.y += y + rand_xy(mt);
  std::normal_distribution<> rand_yaw(0, odom_yaw_noise);
  quaternionTFToMsg(tf::createQuaternionFromYaw(get_yaw(pose.pose.orientation) + yaw + rand_yaw(mt)), pose.pose.orientation); 
}
