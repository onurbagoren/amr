#ifndef PLANNER_XY_H
#define PLANNER_XY_H
using namespace std;

#include <iostream>
#include <vector>
#include <deque>

#include"ros/ros.h"

#include "planner/Node.h"

#include "mapper/mapper.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

using namespace std;

class PlannerXY {
	public:
		PlannerXY();
		virtual ~PlannerXY();
		
		// new subscriber callback functions
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_goal( const geometry_msgs::Pose::ConstPtr& msg );

		bool search( const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const double& w );
		void expand( std::deque< Node* >& open, std::vector< Node* >& closed, Node* node, Node* goal );
		nav_msgs::Path generate_path( void );
		std_msgs::UInt32 open_list_size( void );
		bool check_map( const double& x, const double& y );
		
		// additional class member variables for publishing path, openlistsize,
		// and closedlistsize and storing the nav\_msgs::Odometry message.
		ros::Publisher path_publisher;
		ros::Publisher openlistsize_publisher;
		ros::Publisher closedlistsize_publisher;
		nav_msgs::Odometry _estimated_odometry;
		nav_msgs::Path optimal_path;
		double startX;
		double startY;
		double goalX;
		double goalY;
		double discretization;
		
		bool contains(vector<Node*> v, Node* n);
		
		vector< Node* > openList;
		vector< Node* > closedList;
		vector< Node* > path;
		
		Mapper mapper;

};
#endif /* PLANNER_XY_H */
