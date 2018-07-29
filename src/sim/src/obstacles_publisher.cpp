#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
using namespace std;
int main( int argc, char* argv[] ){
	geometry_msgs::Polygon obstacles;
	
	// setup obstacles
	obstacles.points.push_back( geometry_msgs::Point32() );
	obstacles.points.back().x = 1.5;
	obstacles.points.back().y = 1.5;
	obstacles.points.back().z = 1.0;
	obstacles.points.push_back( geometry_msgs::Point32() );
	obstacles.points.back().x = -1.5;
	obstacles.points.back().y = 1.5;
	obstacles.points.back().z = 0.5;
	obstacles.points.push_back( geometry_msgs::Point32() );
	obstacles.points.back().x = -1.5;
	obstacles.points.back().y = -1.5;
	obstacles.points.back().z = 0.5;
	obstacles.points.push_back( geometry_msgs::Point32() );
	obstacles.points.back().x = 1.5;
	obstacles.points.back().y = -1.5;
	obstacles.points.back().z = 0.25;
	
	ros::init( argc, argv, "obstacles_publisher_node" );
	ros::NodeHandle node_handle;
	ros::Publisher obstacles_publisher = node_handle.advertise< geometry_msgs::Polygon >( "obstacles", 1, true );
	sleep( 1 );
	obstacles_publisher.publish( obstacles );
	sleep( 1 );
	return 0;
}
