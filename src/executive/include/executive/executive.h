#ifndef EXECUTIVE_H
#define EXECUTIVE_H
#include <vector>
#include <deque>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h" // new header file

using namespace std;

class Executive{
	public:
		Executive();
		virtual ~Executive();
		
		nav_msgs::Odometry odometry;
		nav_msgs::Odometry _estimated_odometry;
		vector< geometry_msgs::Pose > _waypoints;
		void create_waypoints( void );
		//void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_path( const nav_msgs::Path::ConstPtr& msg ); 
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void check_waypoints( void );
		int index;
		
		ros::Publisher& goal_publisher( void ){ return _goal_publisher; };
		ros::Subscriber odom_subscriber;
		ros::Publisher _goal_publisher;
		
		
	protected:
		nav_msgs::Odometry _odometry;
		double _distance_threshold;
		double _replan_distance_threshold; // new threshold
		
};

#endif /*EXECUTIVE_H*/
