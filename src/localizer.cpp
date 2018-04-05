#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <random>

class Particle
{
public:
  Particle(void);

  void initialize(int, int, double, geometry_msgs::Pose, int, double, double, double);
  void move(double, double, double);//"odom"から見た"base_link"の動き
  void initialize_map(void);

  geometry_msgs::PoseStamped pose;
  double likelihood;

private:

};

//パラメータ
int N;//number of partcles
double POSITION_SIGMA;
double ORIENTATION_SIGMA;
double INIT_X_COV;
double INIT_Y_COV;
double INIT_YAW_COV;
double INIT_X;
double INIT_Y;
double INIT_YAW;
double ODOM_YAW_NOISE;
double ODOM_X_NOISE;
double ODOM_Y_NOISE;
double RANGE_MAX;
int MATCHING_STEP;
double UPDATE_DISTANCE;
double UPDATE_ANGLE;
double ALPHA_SLOW;
double ALPHA_FAST;
nav_msgs::OccupancyGrid map;
bool map_subscribed = false;
std::vector<Particle>  particles;
geometry_msgs::PoseArray poses;
tf::StampedTransform current_base_link_pose;
tf::StampedTransform previous_base_link_pose;
sensor_msgs::LaserScan laser_data_from_scan;
geometry_msgs::PoseWithCovarianceStamped estimated_pose;
bool calculate_flag = true;
double distance_sum = 0;
double angle_sum = 0;

double w_slow = 0;
double w_fast = 0;

//パーティクル配置用
std::random_device rnd;
std::mt19937 mt(rnd());

double get_yaw(geometry_msgs::Quaternion);
int get_grid_data(double, double);
int get_index(double, double);
int get_i_from_x(double);
int get_j_from_y(double);
double get_square(double);
bool map_valid(int, int);
double get_range_from_map(int, double, double, double);
void initialize_particles(double, double, double);
void initialize_particles_map(void);
void calculate_covariance(void);
double get_larger(double, double);

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data_from_scan = *msg;
  for(int i=0;i<720;i++){
    if(std::isinf(laser_data_from_scan.ranges[i])){
      laser_data_from_scan.ranges[i] = RANGE_MAX;
    }else if(laser_data_from_scan.ranges[i] > RANGE_MAX){
      laser_data_from_scan.ranges[i] = RANGE_MAX;
    }
  }
}

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;

  initialize_particles(INIT_X, INIT_Y, INIT_YAW);
  //initialize_particles_map();

  map_subscribed = true;
}

void init_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  INIT_X = msg->pose.pose.position.x;
  INIT_Y = msg->pose.pose.position.y;
  INIT_YAW = get_yaw(msg->pose.pose.orientation);
  initialize_particles(INIT_X, INIT_Y, INIT_YAW);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    local_nh.getParam("N", N);
    local_nh.getParam("POSITION_SIGMA", POSITION_SIGMA);
    local_nh.getParam("ORIENTATION_SIGMA", ORIENTATION_SIGMA);
    local_nh.getParam("INIT_X_COVARIANCE", INIT_X_COV);
    local_nh.getParam("INIT_Y_COVARIANCE", INIT_Y_COV);
    local_nh.getParam("INIT_YAW_COVARIANCE", INIT_YAW_COV);
    local_nh.getParam("INIT_X", INIT_X);
    local_nh.getParam("INIT_Y", INIT_Y);
    local_nh.getParam("INIT_YAW", INIT_YAW);
    local_nh.getParam("ODOM_X_NOISE", ODOM_X_NOISE);
    local_nh.getParam("ODOM_Y_NOISE", ODOM_Y_NOISE);
    local_nh.getParam("ODOM_YAW_NOISE", ODOM_YAW_NOISE);
    local_nh.getParam("RANGE_MAX", RANGE_MAX);
    local_nh.getParam("MATCHING_STEP", MATCHING_STEP);
    local_nh.getParam("UPDATE_DISTANCE", UPDATE_DISTANCE);
    local_nh.getParam("UPDATE_ANGLE", UPDATE_ANGLE);
    local_nh.getParam("ALPHA_SLOW", ALPHA_SLOW);
    local_nh.getParam("ALPHA_FAST", ALPHA_FAST);

    std::srand(time(NULL));

    ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

    ros::Publisher poses_pub= nh.advertise<geometry_msgs::PoseArray>("/chibi18/poses", 100);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/chibi18/estimated_pose", 100);

    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/chibi18/debug/scan", 100);

    ros::Subscriber init_sub = nh.subscribe("/initialpose", 100, init_callback);

    tf::TransformBroadcaster map_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(INIT_YAW), tf::Vector3(INIT_X, INIT_Y, 0)), ros::Time::now(), "map", "odom");

    estimated_pose.pose.pose.position.x = INIT_X;
    estimated_pose.pose.pose.position.y = INIT_Y;
    estimated_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(INIT_YAW);

    current_base_link_pose = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(INIT_YAW), tf::Vector3(INIT_X, INIT_Y, 0)), ros::Time::now(), "odom", "base_link");
    previous_base_link_pose = current_base_link_pose;
    ros::Rate loop_rate(10);

    while(ros::ok()){
      ros::Time begin = ros::Time::now();
      if(map_subscribed && !laser_data_from_scan.ranges.empty()){
        //prediction
        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;
        geometry_msgs::Quaternion qc, qp;
        try{
          //std::cout << "lookup transform odom to base_link" << std::endl;
          listener.lookupTransform("odom", "base_link", ros::Time(0), current_base_link_pose);
          dx = current_base_link_pose.getOrigin().x() - previous_base_link_pose.getOrigin().x();
          dy = current_base_link_pose.getOrigin().y() - previous_base_link_pose.getOrigin().y();
          //std::cout << "calc quat" << std::endl;
          quaternionTFToMsg(current_base_link_pose.getRotation(), qc);
          quaternionTFToMsg(previous_base_link_pose.getRotation(), qp);
          dtheta = get_yaw(qc) - get_yaw(qp);
          previous_base_link_pose = current_base_link_pose;
          //std::cout << "calc end" << std::endl;
        }catch(tf::TransformException &ex){
          std::cout << ex.what() << std::endl;
        }
        distance_sum += fabs(dx);
        angle_sum += fabs(dtheta);
        if(distance_sum > UPDATE_DISTANCE){
          distance_sum = 0;
          calculate_flag = true;
        }else if(angle_sum > UPDATE_ANGLE){
          angle_sum = 0;
          calculate_flag = true;
        }
        for(int i=0;i<particles.size();i++){
          particles[i].move(dx, dy, dtheta);
        }
        if(calculate_flag){
          //measurement & likelihood
          std::cout << "calculate likelihood" << std::endl;
          sensor_msgs::LaserScan laser_data_from_map;
          laser_data_from_map = laser_data_from_scan;
          laser_data_from_map.header.frame_id = "map";
          for(int i=0;i<N;i++){
            double p_yaw = get_yaw(particles[i].pose.pose.orientation);
            for(int angle=0;angle<720;angle+=MATCHING_STEP){
              laser_data_from_map.ranges[angle] = get_range_from_map(angle, particles[i].pose.pose.position.x, particles[i].pose.pose.position.y, p_yaw);
            }
            //laser_pub.publish(laser_data_from_map);
            double rss = 0;//残差平方和
            for(int angle=0;angle<720;angle+=MATCHING_STEP){
              rss += get_square(laser_data_from_map.ranges[angle] - laser_data_from_scan.ranges[angle]);
            }
            particles[i].likelihood =  exp(-rss / get_square(POSITION_SIGMA) / 2.0);

            //std::cout << rss << ", " << particles[i].likelihood << std::endl;
          }
          double sum = 0;
          for(int i=0;i<N;i++){
            sum += particles[i].likelihood;
          }
          int max_index = 0;
          double w_average = 0;
          for(int i=0;i<N;i++){
            w_average += particles[i].likelihood / (double)N;
            particles[i].likelihood /= sum;
            if(particles[i].likelihood > particles[max_index].likelihood){
              max_index = i;
            }
          }
          if(std::isnan(w_average) || w_average == 0.0){
            w_average = 1.0 / (double)N;
            w_slow = w_fast = w_average;
          }
          if(w_slow == 0.0){
            w_slow = w_average;
          }else{
            w_slow +=  ALPHA_SLOW * (w_average - w_slow);
          }
          if(w_fast == 0.0){
            w_fast = w_average;
          }else{
            w_fast +=  ALPHA_FAST * (w_average - w_fast);
          }
          std::cout << "w_slow:" << w_slow << ", w_fast:" << w_fast << ", w_ave:" << w_average << std::endl;
          /*
          double ess = 0;//有効サンプルサイズ
          for(int i=0;i<N;i++){
            ess += 1 / get_square(particles[i].likelihood);
          }
          std::cout << "ess:" << ess << std::endl;
          */

          //resampling
          std::cout << "resampling" << std::endl;
          std::vector<Particle> new_particles;
          std::uniform_int_distribution<int> rand_n(0, N);
          double beta = 0;
          int index = rand_n(mt);
          int random_count = 0;//for debug
          for(int i=0;i<N;i++){
            double random = rand_n(mt) / (double)N;
            if(get_larger(0.0, 1.0 - w_fast / w_slow) > random){
              //std::cout << "add random particle" << std::endl;
              random_count++;
              Particle p;
              p.initialize(map.info.width, map.info.height, map.info.resolution, map.info.origin, N, estimated_pose.pose.pose.position.x, estimated_pose.pose.pose.position.y, get_yaw(estimated_pose.pose.pose.orientation));
              new_particles.push_back(p);
            }else{
              beta += (rand_n(mt)) / (double)N * 2 * particles[max_index].likelihood;
              while(beta > particles[index].likelihood){
                beta -= particles[index].likelihood;
                index = (1 + index) % N;
              }
              new_particles.push_back(particles[index]);
            }
          }
          std::cout << random_count << " random particles added" << std::endl;

          particles = new_particles;

          for(int i=0;i<N;i++){
            poses.poses[i] = particles[i].pose.pose;
          }
        }
        //odomの補正を計算
        //std::cout << "modfy frame odom" << std::endl;
        try{
          if(calculate_flag){
            calculate_flag = false;
            //推定値の算出
            //std::cout << "calculate estimated_pose" << std::endl;
            int max_index = 0;
            for(int i=0;i<N;i++){
              if(particles[i].likelihood > particles[max_index].likelihood){
                max_index = i;
              }
            }
            std::cout << "i:" << max_index << ", " << particles[max_index].likelihood << std::endl;
            estimated_pose.header.frame_id = "map";
            estimated_pose.pose.pose = particles[max_index].pose.pose;
            //calculate_covariance();
            pose_pub.publish(estimated_pose);

            tf::StampedTransform _transform;
            _transform.stamp_ = ros::Time::now();
            _transform.setOrigin(tf::Vector3(estimated_pose.pose.pose.position.x, estimated_pose.pose.pose.position.y, 0.0));
            _transform.setRotation(tf::createQuaternionFromYaw(get_yaw(estimated_pose.pose.pose.orientation)));
            tf::Stamped<tf::Pose> tf_stamped(_transform.inverse(), laser_data_from_scan.header.stamp, "base_link");
            tf::Stamped<tf::Pose> odom_to_map;
            listener.transformPose("odom", tf_stamped, odom_to_map);
            tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
            temp_tf_stamped = tf::StampedTransform(latest_tf.inverse(), laser_data_from_scan.header.stamp, "map", "odom");

          }
          temp_tf_stamped.stamp_ = ros::Time::now();
          //パーティクルをpublish
          poses_pub.publish(poses);
          //tf
          map_broadcaster.sendTransform(temp_tf_stamped);
        }catch(tf::TransformException ex){
          std::cout << "braodcast error!" << std::endl;
          std::cout << ex.what() << std::endl;
        }
        //std::cout << "from map to odom transform broadcasted" << std::endl;

        //std::cout << "loop:" << ros::Time::now() - begin << "[s]" << std::endl;
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

int get_grid_data(double x, double y)
{
  int data = map.data[get_index(x, y)];
  return data;
}

int get_index(double x, double y)
{
  int index = map.info.width * get_j_from_y(y) + get_i_from_x(x);
  //std::cout << index << " " << x << " " << y <<std::endl;
  return index;
}

int get_i_from_x(double x)
{
  return floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
}

int get_j_from_y(double y)
{
  return floor((y - map.info.origin.position.y) / map.info.resolution + 0.5);
}

Particle::Particle(void)
{
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
}

void Particle::initialize(int width, int height, double resolution, geometry_msgs::Pose origin, int N, double x, double y, double yaw)
{
  std::normal_distribution<> rand_x(x, INIT_X_COV);
  pose.pose.position.x = rand_x(mt);
  std::normal_distribution<> rand_y(y, INIT_Y_COV);
  pose.pose.position.y = rand_y(mt);
  std::normal_distribution<> rand_yaw(yaw, INIT_YAW_COV);
  quaternionTFToMsg(tf::createQuaternionFromYaw(rand_yaw(mt)), pose.pose.orientation);
  likelihood = 1.0 / (double)N;
}

void Particle::initialize_map(void)
{
  std::uniform_int_distribution<int> dist(0, map.info.width);
  pose.pose.position.x = dist(mt) * map.info.resolution + map.info.origin.position.x;
  pose.pose.position.y = dist(mt) * map.info.resolution + map.info.origin.position.y;
  std::uniform_int_distribution<int> dist_yaw(0, 360);
  quaternionTFToMsg(tf::createQuaternionFromYaw(dist_yaw(mt) / 180.0 * M_PI), pose.pose.orientation);
  likelihood = 1.0 / (double)N;
}

void Particle::move(double dx, double dy, double dtheta)
{
  double yaw = get_yaw(pose.pose.orientation);
  std::normal_distribution<> rand_x(0, ODOM_X_NOISE);
  std::normal_distribution<> rand_y(0, ODOM_Y_NOISE);
  std::normal_distribution<> rand_yaw(0, ODOM_YAW_NOISE);
  double distance = sqrt(dx * dx + dy * dy);
  pose.pose.position.x += distance * cos(yaw) + rand_x(mt);
  pose.pose.position.y += distance * sin(yaw) + rand_y(mt);
  quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + dtheta + rand_yaw(mt)), pose.pose.orientation);
}

double get_square(double value)
{
  return value * value;
}

bool map_valid(int i, int j)
{
  return (i>=0) && (i<map.info.width) && (j>=0) && (j<map.info.height);
}


double get_range_from_map(int angle, double ox, double oy, double yaw)
{
  int index0 = get_index(ox, oy);
  int x0 = index0 % map.info.width;
  int y0 = (index0 - x0) / map.info.width;
  int x = 0, y = 0;
  double _angle = angle*laser_data_from_scan.angle_increment - M_PI/2.0;
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

void initialize_particles(double x, double y, double yaw)
{
  particles.clear();
  poses.poses.clear();
  for(int i=0;i<N;i++){
    Particle p;
    do{
      p.initialize(map.info.width, map.info.height, map.info.resolution, map.info.origin, N, x, y, yaw);
    }while(get_grid_data(p.pose.pose.position.x, p.pose.pose.position.y) != 0);
    particles.push_back(p);
    poses.poses.push_back(p.pose.pose);
  }
  poses.header.frame_id = "map";
}

void initialize_particles_map(void)
{
  particles.clear();
  poses.poses.clear();
  for(int i=0;i<N;i++){
    Particle p;
    do{
      p.initialize_map();
    }while(get_grid_data(p.pose.pose.position.x, p.pose.pose.position.y) != 0);
    particles.push_back(p);
    poses.poses.push_back(p.pose.pose);
    std::cout << p.pose.pose.position.x << ", " << p.pose.pose.position.y << std::endl;
  }
  poses.header.frame_id = "map";
}

void calculate_covariance(void)
{
  double mean_x = 0;
  double mean_y = 0;
  double mean_yaw = 0;
  for(int i=0;i<estimated_pose.pose.covariance.size();i++){
    estimated_pose.pose.covariance[i] = 0;
  }
  for(int i=0;i<N;i++){
    mean_x = particles[i].pose.pose.position.x;
    mean_y = particles[i].pose.pose.position.y;
    mean_yaw = get_yaw(particles[i].pose.pose.orientation);
  }
  mean_x /= (double)N;
  mean_y /= (double)N;
  mean_yaw /= (double)N;
  for(int i=0;i<N;i++){
    double _x = particles[i].pose.pose.position.x - mean_x;
    double _y = particles[i].pose.pose.position.y - mean_y;
    double _yaw = get_yaw(particles[i].pose.pose.orientation) - mean_yaw;
    estimated_pose.pose.covariance[0] += _x * _x;
    estimated_pose.pose.covariance[1] += _x * _y;
    estimated_pose.pose.covariance[5] += _x * _yaw;
    estimated_pose.pose.covariance[7] += _y * _y;
    estimated_pose.pose.covariance[11] += _y * _yaw;
    estimated_pose.pose.covariance[35] += _yaw * _yaw;
  }
  estimated_pose.pose.covariance[0] /= (double)N;
  estimated_pose.pose.covariance[1] /= (double)N;
  estimated_pose.pose.covariance[5] /= (double)N;
  estimated_pose.pose.covariance[7] /= (double)N;
  estimated_pose.pose.covariance[11] /= (double)N;
  estimated_pose.pose.covariance[35] /= (double)N;
  estimated_pose.pose.covariance[6] = estimated_pose.pose.covariance[1];
  estimated_pose.pose.covariance[30] = estimated_pose.pose.covariance[5];
  estimated_pose.pose.covariance[31] = estimated_pose.pose.covariance[11];
}

double get_larger(double a, double b)
{
  if(a >= b){
    return a;
  }else{
    return b;
  }
}
