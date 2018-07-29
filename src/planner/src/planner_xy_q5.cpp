#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "planner/planner_xy.h"
#include "geometry_msgs/Pose.h"

using namespace std;

int main(int argc, char* argv[]){
	PlannerXY planner;	
	
	ros::init( argc, argv, "planner_q5" );
	ros::NodeHandle node_handle;
	ros::Publisher goal_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal", 1, true );
	sleep( 1 );
	cout << "creating message" << endl;

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;
	start.position.x = -2.0;
	start.position.y = -3.0;
	start.position.z = 0.0;
	start.orientation.x = 0.0;
	start.orientation.y = 0.0;
	start.orientation.z = 0.0;

	cout << "Start:" << endl << start;
	cout << "publishing message" << endl;
	goal_publisher.publish( start );

	goal.position.x = 7.0;
	goal.position.y = 3.0;
	goal.position.z = 0.0;
	goal.orientation.x = 0.0;
	goal.orientation.y = 0.0;
	goal.orientation.z = 0.0;

	cout << "Goal:" << endl << goal;
	cout << "publishing message" << endl;
	goal_publisher.publish( start );
	sleep(1);
	cout << "done" << endl;
	double w = 1.1;
	cout<< "reached = " << planner.search(start, goal, w) << endl;
	//nav_msgs::Path generate_path();
	//std_msgs::UInt32 open_list_size();
}
