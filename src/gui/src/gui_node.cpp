#include <QtGui/QApplication>
#include "gui/gui.h"
#include "ros/ros.h"

using namespace std;

int
main( int argc, char* argv[] ){
	QApplication app( argc, argv );
	ros::init( argc, argv, "gui" );
	ros::NodeHandle node_handle;
	GUI gui;
	ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "laserscan", 1, &GUI::handleLaserScan, &gui );
	ros::Subscriber subscriber_odom = node_handle.subscribe( "odom", 1, &GUI::handleOdom, &gui );
	ros::Subscriber subscriber_estimated_odom = node_handle.subscribe( "estimated_odom", 1, &GUI::handleEstimatedOdom, &gui );
	ros::Subscriber subscriber_goal = node_handle.subscribe( "goal", 1, &GUI::handleGoal, &gui );
	ros::Subscriber subscriber_lookahead = node_handle.subscribe( "lookahead", 1, &GUI::handleLookahead, &gui );
	ros::Subscriber subscriber_path = node_handle.subscribe( "path", 1, &GUI::handlePath, &gui );
	ros::Subscriber subscriber_projection = node_handle.subscribe( "projection", 1, &GUI::handleProjection, &gui );
	ros::Subscriber subscriber_landmarks = node_handle.subscribe( "landmarks", 1, &GUI::handleLandmarks, &gui );
	ros::Subscriber subscriber_observations = node_handle.subscribe( "observations", 1, &GUI::handleObservations, &gui );
	ros::Subscriber subscriber_scan = node_handle.subscribe( "scan", 1, &GUI::handleLaserScan, &gui );
	ros::Subscriber subscriber_map = node_handle.subscribe( "map", 1, &GUI::handleMap, &gui );
	
	gui.show();
	
	return app.exec();
}
