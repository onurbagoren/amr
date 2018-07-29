#include <iostream>
#include "executive/executive.h"


using namespace std;

Executive::
Executive() :  _waypoints(), _estimated_odometry(), _distance_threshold( 0.1 ), _replan_distance_threshold( 1.0 ){

}

Executive::
~Executive(){

}


void Executive::create_waypoints(){

	vector< double > x_pos;
	vector< double > y_pos;
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	x_pos.push_back(-2.0);
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	x_pos.push_back(-2.0);
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	x_pos.push_back(0.0);
	
	y_pos.push_back(-2.0);
	y_pos.push_back(1.5);
	y_pos.push_back(1.5);
	y_pos.push_back(1.5);
	y_pos.push_back(0.0);
	y_pos.push_back(-2.0);
	y_pos.push_back(1.5);
	y_pos.push_back(1.5);
	y_pos.push_back(1.5);
	y_pos.push_back(0.0);
	y_pos.push_back(-2.0);

	
	for( unsigned int i = 0; i < x_pos.size(); i++ ){
	
		_waypoints.push_back( geometry_msgs::Pose() );
		_waypoints.back().position.x = x_pos.at(i);
		_waypoints.back().position.y = y_pos.at(i);
		
	}
	
	return;
	
}

void Executive::check_waypoints( void ){
	if(index == 0 ){
		_goal_publisher.publish( _waypoints.at( 0 ) );
		index++;
	} else if( abs( _estimated_odometry.pose.pose.position.x - _waypoints.at( index - 1 ).position.x ) < 0.1 && abs( _estimated_odometry.pose.pose.position.y - _waypoints.at( index - 1 ).position.y ) < 0.1 ){		
		_goal_publisher.publish( _waypoints.at( index ) );
		index++;
	}
	
	return;
}

/*void Executive::handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
	odometry = *msg;
	return;
}*/

void
Executive::
handle_path( const nav_msgs::Path::ConstPtr& msg ){
	if( index != _waypoints.size() ){
		if( index == 0 ){
			double dx = _estimated_odometry.pose.pose.position.x - _waypoints.at( 0 ).position.x;
			double dy = _estimated_odometry.pose.pose.position.y - _waypoints.at( 0 ).position.y;
			if( sqrt( dx * dx + dy * dy ) > _replan_distance_threshold ){
				cout << "republishing goal " << _waypoints.at( 0 ).position << endl;
				geometry_msgs::Pose msg;
				msg.position.x = _waypoints.at( 0 ).position.x;
				msg.position.y = _waypoints.at( 0 ).position.y;
				_goal_publisher.publish( msg );
			}
		}else{
			double dx = _estimated_odometry.pose.pose.position.x - _waypoints.at( index ).position.x;
			double dy = _estimated_odometry.pose.pose.position.y - _waypoints.at( index ).position.y;
			if( sqrt( dx * dx + dy * dy ) > _replan_distance_threshold ){
				cout << "republishing goal " << _waypoints.at( index ).position << endl;
				geometry_msgs::Pose msg;
				msg.position.x = _waypoints.at( index ).position.x;
				msg.position.y = _waypoints.at( index ).position.y;
				_goal_publisher.publish( msg );
			}
		}
	}
	return;
}

void
Executive::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	cout << "in handleEstimatedOdom" << endl;
	_estimated_odometry = *msg;
	return;
}
