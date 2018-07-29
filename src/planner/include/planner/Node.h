using namespace std;

#include <iostream>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/UInt32.h>

struct Node{
	public:
		
		double x,y;
		double g;
		double h;
		double f;
		Node* pred;
		int id_x;
		int id_y;
		vector< Node* > children;
		bool operator<(const Node& rhs) const{ // overriding the  "<" operator for Nodes so that when nodes are compared, their f values are compared to one another
			return f < rhs.f;
		}
		bool operator==(const Node& rhs) const{ // overriding the  "==" operator for Nodes so that when nodes are compared, their id_x and id_y values are compared to one another
			if(x == rhs.x && y == rhs.y){
				return true;
			}else{
				return false;
			}	
		}
		bool not_in_open;
		vector< Node* > not_in_open_nodes;	
		~Node(){}		
};


