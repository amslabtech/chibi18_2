#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

geometry_msgs::PointStamped start;
geometry_msgs::PointStamped goal;

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
    heuristic = -1;
    step = 0;
    sum = -1;
    parent_index = -1;
    is_available = -1;
  }

  int cost;
  int step;
  int heuristic;
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
void calculate_heuristic(int);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  std::cout << map.data.size() << std::endl;
  cells.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height*map.info.width;i++){
    cells[i].is_wall = (map.data[i]!=0);
    cells[i].is_available = ((int)map.data[i] > -1);
    //std::cout << "map.data=" << cells[i].is_available << std::endl;
    if(cells[i].is_wall){
      cells[i].cost = 100;
    }
  }
  _cost_map.header = map.header;
  _cost_map.info = map.info;
  _cost_map.data.resize(map.info.height*map.info.width);
  /*
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << cells[j*map.info.width+i].is_wall << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  */
  //calculate AStar
  int start_index = get_index(start.point.x, start.point.y);
  int goal_index = get_index(goal.point.x, goal.point.y);
  std::cout << "calculating heuristic" << std::endl;
  calculate_heuristic(goal_index);
  /*
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << std::setw(2) << cells[j*map.info.width+i].heuristic << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  */
  std::cout << "heuristic calculated!" << std::endl;
  std::cout << "calculating path" << std::endl;
  open_list.push_back(start_index);
  cells[open_list[0]].sum = cells[open_list[0]].step + cells[open_list[0]].heuristic;
  while(!open_list.empty()){
    int n_index = open_list[0];
    int n = cells[n_index].step + cells[n_index].heuristic;
    for(int i=0;i<open_list.size();i++){//openlist中で最小の要素を選択
      if(cells[open_list[i]].sum < n){
        n_index = open_list[i];
        n = cells[n_index].sum;
      }
    }
    if(n_index != goal_index){
      close_list.push_back(n_index);//選んだものがゴールでなければcloselistへ
      open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());//openlistから削除
    }else{
      break;
    }
    int _i = n_index % map.info.width;
    int _j = (n_index - _i) / map.info.width;
    int _index;
    if(_j-1>=0){
      _index = (_j-1)*map.info.width+_i;//i, j-1
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
	  cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }
    }
    if(_j+1<map.info.width){
      _index = (_j+1)*map.info.width+_i;//i, j+1
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }
    }
    if(_i+1<map.info.height){
      _index = _j*map.info.width+(_i+1);//i+1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }
      if(_j-1>=0){
        _index = (_j-1)*map.info.width+(_i+1);//i+1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }
      }
      if(_j+1<map.info.width){
        _index = (_j+1)*map.info.width+(_i+1);//i+1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }
      }
    }
    if(_i-1>=0){
      _index = _j*map.info.width+(_i-1);//i-1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }
      if(_j-1>=0){
        _index = (_j-1)*map.info.width+(_i-1);//i-1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }
      }
      if(_j+1<map.info.width){
        _index = (_j+1)*map.info.width+(_i-1);//i-1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + cells[_index].heuristic;
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }
      }
    }
  }
  /*
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << std::setw(2) << cells[j*map.info.width+i].sum << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << std::setw(2) << cells[j*map.info.width+i].parent_index<< " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  */
  int path_index = goal_index;
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.orientation.w = 1;
  path_pose.header.frame_id = "map";
  while(1){
    path_pose.pose.position.x = (path_index % map.info.width) * map.info.resolution + map.info.origin.position.x;
    path_pose.pose.position.y = (path_index - (path_index % map.info.width)) / map.info.width * map.info.resolution + map.info.origin.position.y;
    global_path.poses.push_back(path_pose);
    path_index = cells[path_index].parent_index;
    if(path_index < 0){
      break;
    }
  }
  std::cout << "global path generated!" << std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("START_X", start.point.x);
  local_nh.getParam("START_Y", start.point.y);
  local_nh.getParam("GOAL_X", goal.point.x);
  local_nh.getParam("GOAL_Y", goal.point.y);
  //std::cout << goal.point.x << " " << goal.point.y<<std::endl;
 
  start.header.frame_id = "map";
  goal.header.frame_id = "map";

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100, true);

  ros::Publisher cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/chibi18/cost_map", 100);

  ros::Rate loop_rate(10);

  global_path.header.frame_id = "map";

  while(ros::ok()){
    if(!map.data.empty()){
      for(int i=0;i<_cost_map.data.size();i++){
        _cost_map.data[i] = cells[i].cost;
      }
      cost_pub.publish(_cost_map);
    }
    if(!global_path.poses.empty()){
      path_pub.publish(global_path);
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

void calculate_heuristic(int goal_index)
{
  ros::Time start_time = ros::Time::now();
  int h = 0;
  int _h = 0;//計算省略用
  int _index = 0;//計算省略用
  cells[goal_index].heuristic = 0;
  bool flag = false;
  while(1){
    flag = false;
    for(int i=0;i<map.info.height;i++){
      for(int j=0;j<map.info.width;j++){
        if(!cells[j*map.info.width+i].is_available){
          continue;
        }
        if(cells[j*map.info.width+i].heuristic == h){
          _h = h + 1;
          flag = true;
          if(j-1>=0){
            _index = (j-1)*map.info.width+i;
            if(cells[_index].heuristic<0){
              cells[_index].heuristic = _h;//i, j-1
            }
          }
          if(j+1<map.info.width){
            _index = (j+1)*map.info.width+i;
            if(cells[_index].heuristic<0){
              cells[_index].heuristic = _h;//i, j+1
            }
          }
          if(i+1<map.info.height){
            _index = j*map.info.width+(i+1);
            if(cells[_index].heuristic<0){
              cells[_index].heuristic = _h;//i+1, j
            }
            if(j-1>=0){
              _index = (j-1)*map.info.width+(i+1);
              if(cells[_index].heuristic<0){
                cells[_index].heuristic = _h;//i+1, j-1
              }
            }
            if(j+1<map.info.width){
              _index = (j+1)*map.info.width+(i+1);
              if(cells[_index].heuristic<0){
                cells[_index].heuristic = _h;//i+1, j+1
              }
            }
          }
          if(i-1>=0){
            _index = j*map.info.width+(i-1);
            if(cells[_index].heuristic<0){
              cells[_index].heuristic = _h;//i-1, j
            }
            if(j-1>=0){
              _index = (j-1)*map.info.width+(i-1); 
              if(cells[_index].heuristic<0){
                cells[_index].heuristic = _h;//i-1, j-1
              }
            }
            if(j+1<map.info.width){
              _index = (j+1)*map.info.width+(i-1);
              if(cells[_index].heuristic<0){
                cells[_index].heuristic = _h;//i-1, j+1
              }
            }
          }
        }
      }
    }
    if(!flag){
      break;;
    }
    if(h > map.info.width*map.info.height){
      std::cout << "heuristic value error!" << std::endl;
      exit(-1);
    }
    //std::cout << h << std::endl;
    h++;
  }
  std::cout << ros::Time::now()-start_time << "[s]" << std::endl;
}
