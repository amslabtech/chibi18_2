#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;

nav_msgs::OccupancyGrid map;
nav_msgs::Path global_path;

geometry_msgs::PoseWithCovarianceStamped estimated_pose;

std::vector<int> waypoint_list;

double MARGIN_WALL;
double WAYPOINT_DISTANCE;
double TOLERANCE;
double WEIGHT_DATA;
double WEIGHT_SMOOTH;

class Cell
{
public:
  Cell(void)
  {
    is_wall = false;
    cost = 0;
    step = 0;
    sum = -1;
    parent_index = -1;
  }

  int cost;
  int step;
  int sum;
  int parent_index;
  bool is_wall;

};

class GlobalPathPlanner
{
public:
  GlobalPathPlanner(void);

  void calculate_aster(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);
  void publish_local_goal();
  void smooth(nav_msgs::Path&);
  int get_heuristic(int, int);
  void calculate_cost_map(void);
  void map_callback(const nav_msgs::OccupancyGridConstPtr&);
  void goal_callback(const geometry_msgs::PoseStampedConstPtr&);
  void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
  void init_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

private:
  ros::NodeHandle nh;
  ros::Publisher cost_pub;
  ros::Publisher target_pub;
  ros::Publisher stop_pub;
  ros::Subscriber map_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber init_sub;

  nav_msgs::OccupancyGrid cost_map;
  std::vector<Cell> cells;
  std::vector<int> open_list;
  std::vector<int> close_list;

};

int get_grid_data(double, double);
int get_index(double, double);
int get_i_from_x(double);
int get_j_from_y(double);
double get_yaw(geometry_msgs::Quaternion);

bool first_aster = true;
bool pose_subscribed = false;
bool global_path_updated = false;

void GlobalPathPlanner::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  std::cout << map.data.size() << std::endl;
  cost_map.header = map.header;
  cost_map.info = map.info;
  cost_map.data.resize(map.info.height*map.info.width);

  int MARGIN_WALL_STEP = 254 / (MARGIN_WALL / map.info.resolution);
  std::cout << "MARGIN_WALL_STEP:" << MARGIN_WALL_STEP << std::endl;

  std::vector<int> wall_list;

  cells.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height*map.info.width;i++){
    cells[i].is_wall = (map.data[i]!=0);
    cells[i].sum = -1;
    cells[i].parent_index = -1;
    if(cells[i].is_wall){
      wall_list.push_back(i);
      cost_map.data[i] = 254;
      cells[i].cost = 254;
    }
  }
  std::cout << "wall:" <<  wall_list.size() << std::endl;
  int i=0;
  while(ros::ok()){
    int cost = cells[wall_list[i]].cost;
    if(i==wall_list.size()){
      break;
    }
    if(cost < MARGIN_WALL_STEP){
     std::cout << "end" << cost << std::endl;
      break;
    }
    int _i = wall_list[i] % map.info.width;
    int _j = (wall_list[i] - _i) / map.info.height;
    if(_i-1>0){
      int index = (_i-1) + (_j) * map.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_i+1<map.info.width){
      int index = (_i+1) + (_j) * map.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_j-1>0){
      int index = (_i) + (_j-1) * map.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_j+1<map.info.width){
      int index = (_i) + (_j+1) * map.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    i++;
  }
  std::cout << "calculate costmap" << std::endl;
  if(!map.data.empty()){
    calculate_cost_map();
  }
  std::cout << "map callback end" << std::endl;
}

void GlobalPathPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if(!first_aster){
    start = goal;
    goal = *msg;
  }else{
    goal = *msg;
    first_aster = false;
  }
  calculate_aster(start, goal);
}

void GlobalPathPlanner::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  estimated_pose = *msg;
  pose_subscribed = true;
}

void GlobalPathPlanner::init_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  start.pose = msg->pose.pose;
  first_aster = true;
  global_path.poses.clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("START_X", start.pose.position.x);
  local_nh.getParam("START_Y", start.pose.position.y);
  local_nh.getParam("WAYPOINT_DISTANCE", WAYPOINT_DISTANCE);
  local_nh.getParam("MARGIN_WALL", MARGIN_WALL);
  local_nh.getParam("TOLERANCE", TOLERANCE);
  local_nh.getParam("WEIGHT_DATA", WEIGHT_DATA);
  local_nh.getParam("WEIGHT_SMOOTH", WEIGHT_SMOOTH);
  //std::cout << goal.pose.position.x << " " << goal.pose.position.y<<std::endl;
  start.pose.orientation.w = 1;
  goal.pose.orientation.w = 1;

  start.header.frame_id = "map";
  goal.header.frame_id = "map";

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100, true);

  tf::TransformListener listener;

  ros::Rate loop_rate(10);

  GlobalPathPlanner global_path_planner;

  global_path.header.frame_id = "map";

  while(ros::ok()){
    if(!global_path.poses.empty() && !waypoint_list.empty()){
      global_path_planner.publish_local_goal();
      if(global_path_updated){
        path_pub.publish(global_path);
        global_path_updated = false;
      }

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
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

double get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

GlobalPathPlanner::GlobalPathPlanner(void)
{
  map_sub = nh.subscribe("/map", 100, &GlobalPathPlanner::map_callback, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 100, &GlobalPathPlanner::goal_callback, this);
  pose_sub = nh.subscribe("/chibi18/estimated_pose", 100, &GlobalPathPlanner::pose_callback, this);
  init_sub = nh.subscribe("/initialpose", 100, &GlobalPathPlanner::init_callback, this);
  target_pub = nh.advertise<geometry_msgs::PoseStamped>("/chibi18/target", 100, true);
  cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/chibi18/cost_map", 100);
  stop_pub = nh.advertise<std_msgs::Bool>("/chibi18/stop", 100);
}

int GlobalPathPlanner::get_heuristic(int diff_x, int diff_y)
{
  /*
  diff_x = abs(diff_x);
  diff_y = abs(diff_y);
  return diff_x + diff_y;
  if(diff_x > diff_y){
    return diff_x;
  }else{
    return diff_y;
  }
  */
  return sqrt(diff_x*diff_x + diff_y*diff_y);
}

void GlobalPathPlanner::calculate_aster(geometry_msgs::PoseStamped& _start, geometry_msgs::PoseStamped& _goal)
{
  std_msgs::Bool permission;
  permission.data = false;
  stop_pub.publish(permission);

  global_path_updated = true;
  //cells.clear();
  open_list.clear();
  close_list.clear();
  //cells.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height*map.info.width;i++){
    //cells[i].is_wall = (map.data[i]!=0);
    cells[i].sum = -1;
    cells[i].parent_index = -1;
    if(cells[i].is_wall){
      //cells[i].cost = 100;
    }
  }

  int start_index = get_index(_start.pose.position.x, _start.pose.position.y);
  int start_i = get_i_from_x(start.pose.position.x);
  int start_j = get_j_from_y(start.pose.position.y);
  int goal_index = get_index(_goal.pose.position.x, _goal.pose.position.y);
  int goal_i = get_i_from_x(goal.pose.position.x);
  int goal_j = get_j_from_y(goal.pose.position.y);
  std::cout << "calculating path" << std::endl;
  std::cout << "from " << _start.pose.position.x << ", " << _start.pose.position.y << ", " << get_yaw(_start.pose.orientation) << ", " << start_index << std::endl;
  std::cout << start_i << ", " << start_j << std::endl;
  std::cout << "to " << _goal.pose.position.x << ", " << _goal.pose.position.y << ", " << get_yaw(_goal.pose.orientation) << ", " << goal_index << std::endl;
  std::cout << goal_i << ", " << goal_j << std::endl;
  open_list.push_back(start_index);
  cells[open_list[0]].sum = cells[open_list[0]].step + get_heuristic(start_i - goal_i, start_j - goal_j);
  while(!open_list.empty() && ros::ok()){
    int n_index = open_list[0];
    int n = cells[n_index].sum;//cells[n_index].step + get_heuristic(goal_i - _i, goal_j - _j);
    for(int i=0;i<open_list.size();i++){//openlist中で最小の要素を選択
      if(cells[open_list[i]].sum < n){
        n_index = open_list[i];
        n = cells[n_index].sum;
      }
    }
    //std::cout << "openlist:" << open_list.size() << std::endl;
    //std::cout << "goal:" << goal_i << ", " << goal_j << std::endl;
    if(n_index != goal_index){
      close_list.push_back(n_index);//選んだものがゴールでなければcloselistへ
      open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());//openlistから削除
    }else{
      break;
    }
    int _index;
    int _i = n_index % map.info.width;
    int _j = (n_index - _i) / map.info.width;
    //std::cout << "current:" << _i << ", " << _j << std::endl;
    //std::cout << "sum:" << cells[n_index].sum << std::endl;
    if(_j-1>=0){
      _index = (_j-1)*map.info.width+_i;//i, j-1
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
	  cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j-1));
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }
    }
    if(_j+1<map.info.width){
      _index = (_j+1)*map.info.width+_i;//i, j+1
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j+1));
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

    }
    if(_i+1<map.info.height){
      _index = _j*map.info.width+(_i+1);//i+1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-_j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*map.info.width+(_i+1);//i+1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j-1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<map.info.width){
        _index = (_j+1)*map.info.width+(_i+1);//i+1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j+1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }

    }
    if(_i-1>=0){
      _index = _j*map.info.width+(_i-1);//i-1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-_j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*map.info.width+(_i-1);//i-1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j-1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<map.info.width){
        _index = (_j+1)*map.info.width+(_i-1);//i-1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j+1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }

    }
  }
  nav_msgs::Path _path;
  _path.header.frame_id = "map";
  int path_index = goal_index;
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.orientation.w = 1;
  path_pose.header.frame_id = "map";
  while(1){
    path_pose.pose.position.x = (path_index % map.info.width) * map.info.resolution + map.info.origin.position.x;
    path_pose.pose.position.y = (path_index - (path_index % map.info.width)) / map.info.width * map.info.resolution + map.info.origin.position.y;
    path_pose.pose.orientation = goal.pose.orientation;
    //std::cout << path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << cells[path_index].cost << ", " << (int)map.data[path_index] << std::endl;
    //std::cout << cells[path_index].cost << std::endl;
    _path.poses.push_back(path_pose);
    path_index = cells[path_index].parent_index;
    //std::cout << "next:" << path_index << std::endl;
    if(path_index < 0){
      std::reverse(_path.poses.begin(), _path.poses.end());
      global_path.poses.insert(global_path.poses.end(), _path.poses.begin(), _path.poses.end());
      waypoint_list.push_back(global_path.poses.size() -1);
      break;
    }
  }
  smooth(global_path);

  permission.data = true;
  stop_pub.publish(permission);

  std::cout << "global path generated!" << std::endl;
}

void GlobalPathPlanner::smooth(nav_msgs::Path& path)
{
  nav_msgs::Path new_path = path;
  double change = TOLERANCE;
  while((change >= TOLERANCE) && (ros::ok())){
    std::cout << "smoothing..." << std::endl;
    change = 0.0;
    for(int i=1;i<path.poses.size() - 1;i++){
      double _x = new_path.poses[i].pose.position.x;
      new_path.poses[i].pose.position.x += WEIGHT_DATA * (path.poses[i].pose.position.x - new_path.poses[i].pose.position.x) + WEIGHT_SMOOTH * (path.poses[i-1].pose.position.x + path.poses[i+1].pose.position.x - 2.0 * new_path.poses[i].pose.position.x);
      change += fabs(_x - new_path.poses[i].pose.position.x);
      double _y = new_path.poses[i].pose.position.y;
      new_path.poses[i].pose.position.y += WEIGHT_DATA * (path.poses[i].pose.position.y - new_path.poses[i].pose.position.y) + WEIGHT_SMOOTH * (path.poses[i-1].pose.position.y + path.poses[i+1].pose.position.y - 2.0 * new_path.poses[i].pose.position.y);
      change += fabs(_y - new_path.poses[i].pose.position.y);
    }
    std::cout << "change:" << change << std::endl;
  }
  path = new_path;
}

void GlobalPathPlanner::calculate_cost_map(void)
{
  for(int i=0;i<cost_map.data.size();i++){
   cost_map.data[i] = cells[i].cost;
  }
  cost_pub.publish(cost_map);
}

void GlobalPathPlanner::publish_local_goal(void)
{
  std::vector<geometry_msgs::PoseStamped>::iterator it = global_path.poses.begin() + waypoint_list[0];
  if(pose_subscribed){
    while((it != global_path.poses.begin()) && (it != global_path.poses.end())){
      //std::cout << global_path.poses.size() << std::endl;
      double error = pow(it->pose.position.x - estimated_pose.pose.pose.position.x, 2) + pow(it->pose.position.y - estimated_pose.pose.pose.position.y, 2);
      if(error < 0){
        std::cout << "pow error" << std::endl;
        break;
      }
      double distance = sqrt(error);
      //std::cout << "d=" << distance << "[m]" << std::endl;
      if(WAYPOINT_DISTANCE > distance){
        target_pub.publish(*it);
        if(distance < WAYPOINT_DISTANCE / 2.0){
          waypoint_list.erase(waypoint_list.begin());
        }
        break;
      }
    --it;
    }
  }
}

