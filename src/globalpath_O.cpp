//globalpath.cpp

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

nav_msgs::OccupancyGrid map;
nav_msgs::Path path;
geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;

struct Grid{
	float h;
	bool obstacle;
	bool open;
	int action[2];	//[0]:x, [1]:y
};

struct OpenList{
	int x;
	int y;
	int g;
	float f;
};

const int height = 4000;
const int width = 4000;
Grid grid[height][width];

int get_index(int i, int j)
{
	return  i*map.info.width+j;
}

bool check_grid(int x, int y)
{
	if(x<0||x>=width)	return false;
	else if(y<0||y>=height)	return false;
	else if(grid[y][x].obstacle)	return false;
	else if(grid[y][x].open)	return false;
	else	return true;
}

/*
int input_list(std::vector<OpenList> list, int x, int y, int g)
{
	bool check = true;
	if(grid[y][x].obstacle)	check = false;
	if(grid[y][x].open)     check = false;
	if(x<0||x>=width)       check = false;
	if(y<0||y>=height)      check = false;
	if(check){
		std::cout << "open" << std::endl;
		list.push_back({x, y, g, g+grid[y][x].h});
		grid[y][x].open = true;
	}
}
*/

int find_minf(std::vector<OpenList>& list)
{
	int i = 0;
	for(int j=0;j<list.size();j++){
		if(list[i].f>list[j].f)	i=j;
	}
	return i;
}

bool check_obstacle(nav_msgs::OccupancyGrid& map, int i, int j)
{
	if(grid[i][j].obstacle) return false;
	
	const int range = 5;
	for(int ii=-range;ii<range;ii++){
		for(int jj=-range;jj<range;jj++){
		if(get_index(i+ii,j+jj)>=0&&get_index(i+ii,j+jj)<map.info.height*map.info.width){
			if(map.data[get_index(i+ii,j+jj)]!=0){
                                        return  true;
				}
			}
		}
	}
	return false;
}


void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{	
	map = *msg;
	std::cout << "finished downloading map" << std::endl;

	//input obstacle and initialize
	for(int i=0;i<map.info.height;i++){
		for(int j=0;j<map.info.width;j++){
			grid[i][j].h = 0.0;
			grid[i][j].open = false;
			grid[i][j].obstacle = false;
			if(check_obstacle(map,i,j))     grid[i][j].obstacle = true;
		}
	}
	std::cout << "finished inputting obstacle" << std::endl;	
	std::cout << "start: " << (int)map.data[get_index(start.pose.position.y, start.pose.position.x)] << std::endl;
	std::cout << "goal:" << (int)map.data[get_index(goal.pose.position.y, goal.pose.position.x)] << std::endl;

	//input huristic
	for(int i=0;i<map.info.height;i++){
		for(int j=0;j<map.info.width;j++){
			int x_dis = goal.pose.position.x-j;
			int y_dis = goal.pose.position.y-i;
			const float dis_parameter = 10.0;	//control how important distanse is
			grid[i][j].h = dis_parameter*sqrt(x_dis*x_dis+y_dis*y_dis);
		}
	}
	std::cout << "finished inputting huristic" << std::endl;

	//A*
	int x = start.pose.position.x;
	int y = start.pose.position.y;
	int g = 0;
	int cost = 0;
	bool found = false;

	std::vector<OpenList> openlist;

	openlist.push_back({x,y,g,g+grid[y][x].h});
	grid[y][x].open = true;

	const int num_action = 8;
	const int action_list[][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};

	while(ros::ok()){
		if(x==goal.pose.position.x&&y==goal.pose.position.y){
			found = true;
			std::cout << "found" << std::endl;
			break;
		}
		if(openlist.empty()){
			std::cout << "failed" << std::endl;
			break;
		}

		g++;

		for(int i=0;i<num_action;i++){
			int x0 = x+action_list[i][0];
			int y0 = y+action_list[i][1];
			if(check_grid(x0, y0)){
				openlist.push_back({x0, y0, g, g+grid[y0][x0].h});
				grid[y0][x0].action[0] = action_list[i][0];
				grid[y0][x0].action[1] = action_list[i][1];
				grid[y0][x0].open = true;
			}
		}
		x = openlist[find_minf(openlist)].x;
		y = openlist[find_minf(openlist)].y;
		openlist.erase(openlist.begin()+find_minf(openlist));
	}
	if(found){	
		while(ros::ok()){		
//			std::cout << x << "," << y << std::endl;	
//			std::cout << grid[y][x].open << ", [" << grid[y][x].action[0] << "," << grid[y][x].action[1] << "]" << std::endl;	

			geometry_msgs::PoseStamped path0;
			path0.header.frame_id = "map";
			path0.pose.position.x = x*map.info.resolution;
			path0.pose.position.y = y*map.info.resolution;
			path0.pose.position.z = 0;
			path0.pose.orientation.x = 0;
			path0.pose.orientation.y = 0;
			path0.pose.orientation.z = 0;
			path0.pose.orientation.w = 1;
			path.poses.push_back(path0);
			if(x==start.pose.position.x&&y==start.pose.position.y){
				std::cout << "completed path" << std::endl;	
				break;
			}
			if(!grid[y][x].open)	break;
			int a=x-grid[y][x].action[0];	
			int b=y-grid[y][x].action[1];
			x = a;
			y = b;	

			//I don't know why these don't work properly
			//x -= grid[y][x].action[0];
			//y -= grid[y][x].action[1];
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "globalpath");
	ros::NodeHandle nh;

	path.header.frame_id = "map";	
	start.header.frame_id = "map";
	goal.header.frame_id = "map";
	start.pose.position.x = (int)(69.4/0.05);
	start.pose.position.y = (int)(100/0.05);
	goal.pose.position.x = (int)(121/0.05);
	goal.pose.position.y = (int)(114/0.05);

	ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/globalpath",100);

	ros::Rate loop_rate(10);
	while(ros::ok()){	
		if(!path.poses.empty())		path_pub.publish(path);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
