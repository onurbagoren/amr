#include <iostream> 
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
using namespace std;
int
main( int argc, char* argv[] ){
	ros::init( argc, argv, "gui_goal_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher goal_publisher = node_handle.advertise< geometry_msgs::Pose>( "goal", 1, true);
	sleep( 1 );
	cout << "creating message" << endl;
	geometry_msgs::Pose msg;
	msg.position.x = -5.0;
	msg.position.y = -4.0;
	msg.position.z = 0.0;
	msg.orientation.x = 0.0;
	msg.orientation.y = 0.0;
	msg.orientation.z = 0.0;
	msg.orientation.w = 1.0;
	cout << "message:" << endl << msg;
	cout << "publishing message" << endl;
	goal_publisher.publish( msg );
	sleep(1);
	cout << "done" << endl;
	return 0;
}
