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
	sleep( 1 );
	cout << "creating message" << endl;

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;
	start.position.x = 5.0;
	start.position.y = 2.0;
	start.position.z = 0.0;
	start.orientation.x = 0.0;
	start.orientation.y = 0.0;
	start.orientation.z = 0.0;
	
	goal.position.x = -3.0;
	goal.position.y = 4.0;
	goal.position.z = 0.0;
	goal.orientation.x = 0.0;
	goal.orientation.y = 0.0;
	goal.orientation.z = 0.0;

	sleep(1);
	double w = 1.5;
	cout << "reached = " << planner.search(start,goal,w) << endl;
	//planner.search(start,goal,w);
	cout << planner.generate_path() << endl;
	//std_msgs::UInt32 a = planner.open_list_size();
	//cout << a.data << endl;
	//cout << planner.open_list_size() << endl;
	
}
