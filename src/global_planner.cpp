#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;

nav_msgs::OccupancyGrid map;
nav_msgs::Path global_path;

nav_msgs::OccupancyGrid _cost_map;//for debug

class Cell 
{
public:
  Cell(void)
  {
    is_wall = false;
    cost = 1;
    //heuristic = -1;
    step = 0;
    sum = -1;
    parent_index = -1;
    is_available = -1;
  }

  int cost;
  int step;
  //int heuristic;
  int sum;
  int parent_index;
  bool is_wall;
  bool is_available;
};

std::vector<Cell> cells;
std::vector<int> open_list;
std::vector<int> close_list;


int get_grid_data(float, float);
int get_index(float, float);
int get_heuristic(int, int);
void calculate_aster(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);
float get_yaw(geometry_msgs::Quaternion);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  std::cout << map.data.size() << std::endl;
  
  _cost_map.header = map.header;
  _cost_map.info = map.info;
  _cost_map.data.resize(map.info.height*map.info.width);

  calculate_aster(start, goal);
}

void goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  start = goal;
  goal = *msg;
  calculate_aster(start, goal); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("START_X", start.pose.position.x);
  local_nh.getParam("START_Y", start.pose.position.y);
  local_nh.getParam("GOAL_X", goal.pose.position.x);
  local_nh.getParam("GOAL_Y", goal.pose.position.y);
  //std::cout << goal.pose.position.x << " " << goal.pose.position.y<<std::endl;
  start.pose.orientation.w = 1;
  goal.pose.orientation.w = 1;
 
  start.header.frame_id = "map";
  goal.header.frame_id = "map";

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100, true);

  ros::Publisher cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/chibi18/cost_map", 100);

  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 100, goal_callback);

  ros::Rate loop_rate(10);

  global_path.header.frame_id = "map";

  while(ros::ok()){
    if(!map.data.empty()){
      /*
      for(int i=0;i<_cost_map.data.size();i++){
        _cost_map.data[i] = cells[i].cost;
      }
      cost_pub.publish(_cost_map);
      */
    }
    if(!global_path.poses.empty()){
      path_pub.publish(global_path);
      //std::cout << get_grid_data(5.05, -1.35) << std::endl;;
      //std::cout << get_index(5.05, -1.35) << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
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
  int index = int((map.info.width*(y-map.info.origin.position.y)+(x-map.info.origin.position.x))/map.info.resolution) + 1;
  //std::cout << index << " " << x << " " << y <<std::endl;
  return index;
}

int get_heuristic(int diff_x, int diff_y)
{
  diff_x = abs(diff_x);
  diff_y = abs(diff_y);
  return diff_x + diff_y;
  if(diff_x > diff_y){
    std::cout << "h:" << diff_x << std::endl;
    return diff_x;
  }else{
    std::cout << "h:" << diff_y << std::endl;
    return diff_y;
  }
}

void calculate_aster(geometry_msgs::PoseStamped& _start, geometry_msgs::PoseStamped& _goal)
{
  cells.clear();
  open_list.clear();
  close_list.clear();
  cells.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height*map.info.width;i++){
    cells[i].is_wall = (map.data[i]!=0);
    cells[i].is_available = ((int)map.data[i] > -1);
    //std::cout << "map.data=" << cells[i].is_available << std::endl;
    if(cells[i].is_wall){
      cells[i].cost = 100;
    }
  }
  int start_index = get_index(_start.pose.position.x, _start.pose.position.y);
  int start_i = start_index % map.info.width;
  int start_j = (start_index - start_i) / map.info.width;
  int goal_index = get_index(_goal.pose.position.x, _goal.pose.position.y);
  int goal_i = goal_index % map.info.width;
  int goal_j = (goal_index - goal_i) / map.info.width;
  std::cout << "calculating path" << std::endl;
  std::cout << "from " << _start.pose.position.x << ", " << _start.pose.position.y << ", " << get_yaw(_start.pose.orientation) << ", " << start_index << std::endl;
  std::cout << "to " << _goal.pose.position.x << ", " << _goal.pose.position.y << ", " << get_yaw(_goal.pose.orientation) << ", " << goal_index << std::endl;
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
    std::cout << "openlist:" << open_list.size() << std::endl;
    std::cout << "goal:" << goal_i << ", " << goal_j << std::endl;
    if(n_index != goal_index){
      close_list.push_back(n_index);//選んだものがゴールでなければcloselistへ
      open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());//openlistから削除
    }else{
      break;
    }
    int _index;
    int _i = n_index % map.info.width;
    int _j = (n_index - _i) / map.info.width;
    std::cout << "current:" << _i << ", " << _j << std::endl;
    std::cout << "sum:" << cells[n_index].sum << std::endl;
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
  int path_index = goal_index;
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.orientation.w = 1;
  path_pose.header.frame_id = "map";
  while(1){
    path_pose.pose.position.x = (path_index % map.info.width) * map.info.resolution + map.info.origin.position.x;
    path_pose.pose.position.y = (path_index - (path_index % map.info.width)) / map.info.width * map.info.resolution + map.info.origin.position.y;
    path_pose.pose.orientation.w = 1;
    //std::cout << path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << cells[path_index].cost << ", " << (int)map.data[path_index] << std::endl;
    //std::cout << cells[path_index].cost << std::endl;
    global_path.poses.push_back(path_pose);
    path_index = cells[path_index].parent_index;
    if(path_index < 0){
      break;
    }
  }
  std::cout << "global path generated!" << std::endl;
}

float get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}
