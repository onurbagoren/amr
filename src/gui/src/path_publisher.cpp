#include <iostream>
#include "ros/ros.h"

#include"nav_msgs/Path.h"

using namespace std;

geometry_msgs::PoseStamped create_pose( const double& x, const double& y ){
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;
	pose_stamped.pose.position.z = 0.0;
	pose_stamped.pose.orientation.x = 0.0;
	pose_stamped.pose.orientation.y = 0.0;
	pose_stamped.pose.orientation.z = 0.0;
	pose_stamped.pose.orientation.w = 1.0;
	return pose_stamped;
}

int main( int argc, char* argv[] ){
	ros::init( argc, argv, "path_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher path_publisher = node_handle.advertise< nav_msgs::Path >( "path", 1, true);
	sleep( 1 );
	cout << "creating message" << endl;
	nav_msgs::Path msg;
	msg.poses.push_back( create_pose( 0.0, 0.0 ) );
	msg.poses.push_back( create_pose( 1.0, 0.0 ) );
	msg.poses.push_back( create_pose( 2.0, 1.0 ) );
	msg.poses.push_back( create_pose( 3.0, 2.0 ) );
	msg.poses.push_back( create_pose( 4.0, 3.0 ) );
	msg.poses.push_back( create_pose( 5.0, 3.0 ) );
	cout << "message:" << endl << msg;
	cout << "publishing message" << endl;
	path_publisher.publish( msg );
	sleep(1);
	cout << "done" << endl;
	return 0;
}
