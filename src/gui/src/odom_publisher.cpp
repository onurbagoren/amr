#include <iostream>
#include "ros/ros.h" 
#include "nav_msgs/Odometry.h"
using namespace std;
int
main( int argc,char* argv[] ){
	ros::init( argc, argv, "odom_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher odom_publisher = node_handle.advertise< nav_msgs::Odometry >("odom", 1, true);
	
	sleep( 1 );

	cout << "creating message" << endl;
	nav_msgs::Odometry msg;
	msg.pose.pose.position.x = 1;
	msg.pose.pose.position.y = 1;
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation.x = 0.0;
	msg.pose.pose.orientation.y = 0.0;
	msg.pose.pose.orientation.z = 0.7071;
	msg.pose.pose.orientation.w = 0.7071;
	cout << "message:" << endl << msg;
	cout << "publishing message" << endl;
	odom_publisher.publish( msg );
	sleep(1);

	cout << "done" << endl;

	return 0;
}
