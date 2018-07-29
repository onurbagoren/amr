#ifndef DRIVE_TO_POINT
#define DRIVE_TO_POINT
#include <iostream>
#include <deque>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
class DriveToPoint {
	public:
		DriveToPoint();
		virtual ~DriveToPoint();
		
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handlePath( const nav_msgs::Path::ConstPtr& msg );
		void updatePathIndex( void );
		void updateCommand( void );
		nav_msgs::Odometry _estimated_odometry;
		nav_msgs::Path path;
		geometry_msgs::Twist command;
		nav_msgs::Path projection;
		geometry_msgs::Point lookahead;
		double	max_speed;
		int	path_index;
		double distance_threshold;
		double angle_threshold;
};

std::ostream& operator<<( std::ostream& out, const DriveToPoint& other );

#endif /* DRIVE_TO_POINT */
