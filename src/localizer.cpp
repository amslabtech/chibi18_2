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
nav_msgs::OccupancyGrid map;
bool map_subscribed = false;
std::vector<Particle>  particles;
geometry_msgs::PoseArray poses;
geometry_msgs::TransformStamped transform;
tf::StampedTransform current_base_link_pose;
tf::StampedTransform previous_base_link_pose;
sensor_msgs::LaserScan data;

float get_yaw(geometry_msgs::Quaternion);
int get_grid_data(float, float);
int get_index(float, float);
void set_transform(float, float, float);

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  data = *msg;
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
    std::cout << N << std::endl;

    std::srand(time(NULL));

    ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

    ros::Publisher poses_pub= nh.advertise<geometry_msgs::PoseArray>("/chibi18/poses", 100);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    tf::TransformBroadcaster map_broadcaster;
    tf::TransformListener listener;

    ros::Rate loop_rate(10);

    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";

    set_transform(42, 32, 0);//適当

    while(ros::ok()){
      if(map_subscribed){
        //pridiction
        float dx, dy, dtheta;
        try{
          listener.lookupTransform("odom", "base_link", ros::Time(0), current_base_link_pose);
          dx = current_base_link_pose.getOrigin().x() - previous_base_link_pose.getOrigin().x();
          dy = current_base_link_pose.getOrigin().y() - previous_base_link_pose.getOrigin().y();
          geometry_msgs::Quaternion qc, qp;
          quaternionTFToMsg(current_base_link_pose.getRotation(), qc);
          quaternionTFToMsg(previous_base_link_pose.getRotation(), qp);
          dtheta = get_yaw(qc) - get_yaw(qp);
        }catch(tf::TransformException &ex){
          std::cout << ex.what() << std::endl;
          continue;
        }
        for(int i=0;i<particles.size();i++){
          //particles[i].move(0.01, 0.01, 0.05);//適当
          particles[i].move(dx, dy, dtheta);
          poses.poses[i] = particles[i].pose.pose;
        }
        //measurement

        //likelihood

        //resampling

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
  pose.pose.position.x = (rand() % width) * resolution + origin.position.x;
  pose.pose.position.y = (rand() % height) * resolution + origin.position.y;
  quaternionTFToMsg(tf::createQuaternionFromYaw((rand() % 360) * M_PI / 180.0 - M_PI), pose.pose.orientation);
  likelihood = 1.0 / (float)N;
}

void Particle::move(float x, float y, float yaw)
{
  pose.pose.position.x += x * cos(get_yaw(pose.pose.orientation)) - y * sin(get_yaw(pose.pose.orientation));
  pose.pose.position.y += x * sin(get_yaw(pose.pose.orientation)) + y * cos(get_yaw(pose.pose.orientation));
  quaternionTFToMsg(tf::createQuaternionFromYaw(get_yaw(pose.pose.orientation) + yaw), pose.pose.orientation); 
}
