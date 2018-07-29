#include <iostream>
#include "ros/ros.h"
#include "mapper/mapper.h"
using namespace std;
int main( int argc, char* argv[] ){

	Mapper mapper( 0.1, 201, 201 );
	ros::init( argc, argv, "mapper_node" );
	ros::NodeHandle node_handle;
	ros::Subscriber odometry_subscriber = node_handle.subscribe( "estimated_odom", 1, &Mapper::handleOdometry, &mapper );
	ros::Subscriber scan_subscriber = node_handle.subscribe( "scan", 1, &Mapper::handleLaserScan, &mapper );
	ros::Publisher map_publisher = node_handle.advertise< map_msgs::OccupancyGridUpdate >( "map", 1, true );
	double frequency = 10.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		mapper.update();
		map_publisher.publish( mapper.map_msg() );
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
	
}
