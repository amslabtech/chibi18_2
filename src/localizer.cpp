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
    local_nh.getParam("POSISION_SIGMA", POSITION_SIGMA);
    local_nh.getParam("ORIENTATION_SIGMA", ORIENTATION_SIGMA);
    local_nh.getParam("INIT_X_COVARIANCE", init_x_cov);
    local_nh.getParam("INIT_Y_COVARIANCE", init_y_cov);
    local_nh.getParam("INIT_YAW_COVARIANCE", init_yaw_cov);
    local_nh.getParam("INIT_X", init_x);
    local_nh.getParam("INIT_Y", init_y);
    local_nh.getParam("INIT_YAW", init_yaw);
    local_nh.getParam("ODOM_XY_NOISE", odom_xy_noise);
    local_nh.getParam("ODOM_YAW_NOISE", odom_yaw_noise);

    std::srand(time(NULL));

    ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

    ros::Publisher poses_pub= nh.advertise<geometry_msgs::PoseArray>("/chibi18/poses", 100);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    tf::TransformBroadcaster map_broadcaster;
    tf::TransformListener listener;

    ros::Rate loop_rate(10);

    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";

    set_transform(init_x, init_y, init_yaw);//適当

    while(ros::ok()){
      if(map_subscribed/* && !laser_data_from_scan.ranges.empty()*/){
        //pridiction
        float dx, dy, dtheta;
        /*
        try{
          listener.lookupTransform("odom", "base_link", ros::Time(0), current_base_link_pose);
          dx = current_base_link_pose.getOrigin().x() - previous_base_link_pose.getOrigin().x();
          dy = current_base_link_pose.getOrigin().y() - previous_base_link_pose.getOrigin().y();
          geometry_msgs::Quaternion qc, qp;
          quaternionTFToMsg(current_base_link_pose.getRotation(), qc);
          quaternionTFToMsg(previous_base_link_pose.getRotation(), qp);
          dtheta = get_yaw(qc) - get_yaw(qp);
          previous_base_link_pose = current_base_link_pose; 
        }catch(tf::TransformException &ex){
          std::cout << ex.what() << std::endl;
          continue;
        }
        */
        for(int i=0;i<particles.size();i++){
          particles[i].move(0.01, 0.01, 0.05);//適当
          //particles[i].move(dx, dy, dtheta);
        }

        /*
        //measurement & likelihood
        pcl::PointCloud<pcl::PointXYZ> pcl_from_scan;
        pcl::fromROSMsg(data_from_scan, pcl_from_scan);
        pcl::PointCloud<pcl::PointXYZ> pcl_from_map;
        sensor_msgs::PointCloud2 data_from_map;
        sensor_msgs::LaserScan laser_data_from_map;
        laser_data_from_map = laser_data_from_scan; 
        for(int i=0;i<N;i++){
          //
          // マップから点を取得する処理を書くこと
          //
          for(float theta=0;theta<M_PI;theta+=M_PI/720.0){
            if(theta<M_PI/2.0){
              
            }else if(theta>M_PI/2.0){

            }else{

            }
          }
          projector.projectLaser(laser_data_from_map, data_from_map);
          pcl::fromROSMsg(data_from_map, pcl_from_scan);

          pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
          icp.setInputSource(pcl_from_scan.makeShared());
          icp.setInputTarget(pcl_from_map.makeShared());

          pcl::PointCloud<pcl::PointXYZ> output;
          icp.align(output);

          Eigen::Matrix4d m = Eigen::Matrix4d::Identity ();
          m = icp.getFinalTransformation ().cast<double>();
          float position_error = sqrt(pow(m(0, 3), 2) + pow(m(1, 3), 2));
          float orientation_error = acos(m(0));
          particles[i].likelihood = exp(-pow(error / POSITION_SIGMA, 2) / 2.0) exp(-pow(orientation_error / ORIENTATION_SIGMA, 2) / 2.0);
        }
        */
        float sum = 0;
        for(int i=0;i<N;i++){
          sum += particles[i].likelihood;
        }
        for(int i=0;i<N;i++){
          particles[i].likelihood /= sum;
        }

        //resampling
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
        transform.header.stamp = ros::Time::now();
        map_broadcaster.sendTransform(transform);
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
  /*
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
  */
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
  pose.pose.position.x += x * cos(get_yaw(pose.pose.orientation)) - y * sin(get_yaw(pose.pose.orientation)) + rand_xy(mt);
  pose.pose.position.y += x * sin(get_yaw(pose.pose.orientation)) + y * cos(get_yaw(pose.pose.orientation)) + rand_xy(mt);
  std::normal_distribution<> rand_yaw(0, odom_yaw_noise);
  quaternionTFToMsg(tf::createQuaternionFromYaw(get_yaw(pose.pose.orientation) + yaw + rand_yaw(mt)), pose.pose.orientation); 
}
