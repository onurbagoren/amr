#include <iostream>
#include "ros/ros.h"
#include "pfc/drive_to_point.h"
using namespace std;

int
main( int argc, char* argv[] ){
	DriveToPoint pfc;
	ros::init( argc, argv, "drive_to_node" );
	ros::NodeHandle node_handle;
	ros::Subscriber odometry_subscriber = node_handle.subscribe( "estimated_odom", 1, &DriveToPoint::handleEstimatedOdom, &pfc );
	ros::Subscriber path_subscriber = node_handle.subscribe( "path", 1, &DriveToPoint::handlePath, &pfc );
	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "cmd_vel_mux/input/navi", 1, true );
	ros::Publisher lookahead_publisher = node_handle.advertise< geometry_msgs::Point >( "lookahead", 1, true );
	ros::Publisher projection_publisher = node_handle.advertise< nav_msgs::Path>( "projection", 1, true );
	
	if( argc > 1 ){
		pfc.max_speed = strtod( argv[1], NULL );
	}
	
	if( argc > 2 ){
		pfc.distance_threshold = strtod( argv[2], NULL );
	}
	
	if( argc > 3 ){
		pfc.angle_threshold = strtod( argv[3], NULL );
	}
	
	cout << "max_speed:" << pfc.max_speed << endl;
	cout << "distance_threshold:" << pfc.distance_threshold << endl;
	cout << "angle_threshold:" << pfc.angle_threshold << endl;
	
	double frequency = 30.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		pfc.updatePathIndex();
		pfc.updateCommand();
		command_publisher.publish( pfc.command );
		lookahead_publisher.publish( pfc.lookahead );
		projection_publisher.publish( pfc.projection );
		ros::spinOnce();
		timer.sleep();
	}
	
	
	return 0;
}
