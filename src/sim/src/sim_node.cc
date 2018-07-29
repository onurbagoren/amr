#include <iostream>
#include "ros/ros.h"
#include "sim/sim.h"
using namespace std;

int
main( int argc, char* argv[] ){
	Sim sim;
	ros::init( argc, argv, "sim_node" );
	ros::NodeHandle node_handle;
	ros::Subscriber command_subscriber = node_handle.subscribe( "cmd_vel_mux/input/navi", 1, &Sim::handle_command, &sim );
	ros::Subscriber landmarks_subscriber = node_handle.subscribe( "landmarks", 1, &Sim::handle_landmarks, &sim );
	ros::Subscriber obstacles_subscriber = node_handle.subscribe( "obstacles", 1, &Sim::handle_obstacles, &sim );
	ros::Publisher odometry_publisher = node_handle.advertise< nav_msgs::Odometry >( "odom", 1, true );
	//ros::Publisher observed_landmarks_publisher = node_handle.advertise< perception::Landmarks >( "observed_landmarks", 1, true ); 
	sim.observations_publisher = node_handle.advertise< perception::Observations >( "observations", 1, true );
	ros::Publisher scan_publisher = node_handle.advertise< sensor_msgs::LaserScan >( "scan", 1, true );
	
	double frequency = 10.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		sim.step( 1.0/frequency );
		sim.observations_determine();
		odometry_publisher.publish( sim.odometry_msg() );
		scan_publisher.publish( sim.scan_msg() );
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
