#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

class Particle
{
public:
  Particle(void)
  {
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
  }

  void initialize(int width, int height, float resolution, geometry_msgs::Pose origin)
  {
    pose.pose.position.x = (rand() % width) * resolution + origin.position.x;
    pose.pose.position.y = (rand() % height) * resolution + origin.position.y;
    quaternionTFToMsg(tf::createQuaternionFromYaw((rand() % 360) * M_PI / 180.0 - M_PI), pose.pose.orientation);
  }

  geometry_msgs::PoseStamped pose;

private:

};

int N;//number of partcles
nav_msgs::OccupancyGrid map;
bool map_subscribed = false;
std::vector<Particle>  particles;
geometry_msgs::PoseArray poses;

float get_yaw(geometry_msgs::Quaternion);
int get_grid_data(float, float);
int get_index(float, float);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  
  for(int i=0;i<N;i++){
    Particle p;
    do{
      p.initialize(map.info.width, map.info.height, map.info.resolution, map.info.origin);
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

    tf::TransformBroadcaster map_broadcaster;
    ros::Rate loop_rate(10);

    while(ros::ok()){
      if(map_subscribed){
        poses_pub.publish(poses);
        map_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"map", "odom"));
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

