#include <iostream>
#include "ros/ros.h"
#include "executive/executive.h"

using namespace std;

int
main( int argc, char* argv[]){
	sleep( 5 );
	Executive exec;
	ros::init( argc, argv, "exec" );
	ros::NodeHandle node_handle;
	
	//ros::Subscriber path_subscriber = node_handle.subscribe( "path", 1, &Executive::handle_path, &exec ); // new subscriber
	//ros::Subscriber waypoints_subscriber = node_handle.subscribe( "waypoints", 1, &Executive::handle_waypoints, &executive );
	
	exec._goal_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal", 1, true );
	exec.create_waypoints();
	//exec.odom_subscriber = node_handle.subscribe( "odom", 1, &Executive::handle_odom, &exec );
	exec.odom_subscriber = node_handle.subscribe( "estimated_odom", 1, &Executive::handleEstimatedOdom, &exec );
	
	
	ros::Rate timer(10);
	exec.index = 0;
	
	while( ros::ok ){
		exec.check_waypoints();
		ros::spinOnce();
		timer.sleep();
	}
	
	return 0;
}

