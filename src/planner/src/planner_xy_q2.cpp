#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "planner/planner_xy.h"
#include "geometry_msgs/Pose.h"

using namespace std;

int main(int argc, char* argv[]){

	PlannerXY planner;
	
	ros::init( argc, argv, "planner_q2" );
	ros::NodeHandle node_handle;
	ros::Publisher plan_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal", 1, true );
	ros::Publisher openlistsize_publisher = node_handle.advertise< std_msgs::UInt32 >( "openlistsize", 1, true );
	ros::Publisher path_publisher = node_handle.advertise< nav_msgs::Path >( "path", 1, true );
	
	sleep( 1 );
	cout << "creating message" << endl;

	geometry_msgs::Pose start;
	start.position.x = 0.0;
	start.position.y = 0.0;
	start.position.z = 0.0;
	start.orientation.x = 0.0;
	start.orientation.y = 0.0;
	start.orientation.z = 0.0;
	
	geometry_msgs::Pose goal;
	goal.position.x = 5.0;
	goal.position.y = 7.0;
	goal.position.z = 0.0;
	goal.orientation.x = 0.0;
	goal.orientation.y = 0.0;
	goal.orientation.z = 0.0;
	
	cout << "-7.5 mod 2pi = " << fmod( -7.5, 2*M_PI) << endl;

	sleep(1);
	double w = 1.0;
	cout << "reached = " << planner.search(start,goal,w) << endl;
	cout << planner.generate_path() << endl;
	plan_publisher.publish( goal );
	planner.open_list_size();
	nav_msgs::Path p = planner.generate_path();
	path_publisher.publish( p );
	//std_msgs::UInt32 a = planner.open_list_size();
	//cout << a.data << endl;
	//planner.openList_c();
	
}
