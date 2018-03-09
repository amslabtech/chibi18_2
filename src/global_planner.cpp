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
  }

  int cost;
  int step;
  int heuristic;
  int parent_index;
  bool is_wall;
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
  cells.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height*map.info.width;i++){
    cells[i].is_wall = (map.data[i]!=0);
    if(cells[i].is_wall){
      cells[i].cost = 100;
    }
  }
  _cost_map.header = map.header;
  _cost_map.info = map.info;
  _cost_map.data.resize(map.info.height*map.info.width);
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << cells[j*map.info.width+i].is_wall << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  //calculate AStar
  int start_index = get_index(start.point.x, start.point.y);
  int goal_index = get_index(goal.point.x, goal.point.y);
  calculate_heuristic(goal_index);
  open_list.push_back(start_index);
  while(!open_list.empty()){
    int n_index = open_list[0];
    int n = cells[n_index].step + cells[n_index].heuristic;
    for(int i=0;i<open_list.size();i++){//openlist中で最小の要素を選択
      if((cells[open_list[i]].step + cells[open_list[i]].heuristic) < n){
        n_index = i;
        n = cells[n_index].step + cells[n_index].heuristic;
      }
    }
    if(n_index != goal_index){
      close_list.push_back(n_index);//選んだものがゴールでなければcloselistへ
    }else{
      break;
    }
  }

  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      std::cout << std::setw(2) << cells[j*map.info.width+i].heuristic << " ";
    }
    std::cout << std::endl;
  }
  open_list.push_back(start_index);
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

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100);

  ros::Publisher cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/chibi18/cost_map", 100);

  ros::Rate loop_rate(10);

  global_path.header.frame_id = "map";


  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation.w = 1;
  global_path.poses.push_back(pose);
  pose.pose.position.x = 1;
  global_path.poses.push_back(pose);

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
  std::cout << index << " " << x << " " << y <<std::endl;
  return index;
}

void calculate_heuristic(int goal_index)
{
  int h = 0;
  cells[goal_index].heuristic = 0;
  bool flag = false;
  while(1){
    flag = false;
    for(int i=0;i<map.info.height;i++){
      for(int j=0;j<map.info.width;j++){
        if(cells[j*map.info.width+i].heuristic == h){
          flag = true;
          if(j-1>=0){
            if(cells[(j-1)*map.info.width+i].heuristic<0){
              cells[(j-1)*map.info.width+i].heuristic = h+1;//i, j-1
            }
          }
          if(j+1<map.info.width){
            if(cells[(j+1)*map.info.width+i].heuristic<0){
              cells[(j+1)*map.info.width+i].heuristic = h+1;//i, j+1
            }
          }
          if(i+1<map.info.height){
            if(cells[j*map.info.width+(i+1)].heuristic<0){
              cells[j*map.info.width+(i+1)].heuristic = h+1;//i+1, j
            }
            if(j-1>=0){
              if(cells[(j-1)*map.info.width+(i+1)].heuristic<0){
                cells[(j-1)*map.info.width+(i+1)].heuristic = h+1;//i+1, j-1
              }
            }
            if(j+1<map.info.width){
              if(cells[(j+1)*map.info.width+(i+1)].heuristic<0){
                cells[(j+1)*map.info.width+(i+1)].heuristic = h+1;//i+1, j+1
              }
            }
          }
          if(i-1>=0){
            if(cells[j*map.info.width+(i-1)].heuristic<0){
              cells[j*map.info.width+(i-1)].heuristic = h+1;//i-1, j
            }
            if(j-1>=0){
              if(cells[(j-1)*map.info.width+(i-1)].heuristic<0){
                cells[(j-1)*map.info.width+(i-1)].heuristic = h+1;//i-1, j-1
              }
            }
            if(j+1<map.info.width){
              if(cells[(j+1)*map.info.width+(i-1)].heuristic<0){
                cells[(j+1)*map.info.width+(i-1)].heuristic = h+1;//i-1, j+1
              }
            }
          }
        }
      }
    }
    if(!flag){
      return;
    }
    h++;
  }
}
