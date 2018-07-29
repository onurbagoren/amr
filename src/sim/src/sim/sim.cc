#include <iostream>
#include "sim/sim.h"
using namespace std;

/**
*x(0) = x position
*x(1) = y position
*x(2) = heading
*u(0) = linear velocity (v)
*u(1) = angular_velocity (w)
*/
geometry_msgs::Quaternion
yaw_to_quaternion(const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion;
}

double
sample(const double& bsquared ){
	double tmp = 0.0;
	for( unsigned int i = 0; i < 12; i++ ){
		int R = -bsquared + 2.0 * bsquared * (double)(rand() % 1000)/(double)(1000);
		tmp = tmp + R;
	}
	tmp = tmp/2.0;
	return tmp;
}

Sim::
Sim() : _x( 0.0, 0.0, 0.0 ), _u( 0.0, 0.0, 0.0 ), _alpha1( 0.005 ), _alpha2( 0.05 ), _alpha3( 0.005 ), _alpha4( 0.05 ), _alpha5( 0.005 ), _alpha6( 0.05 ), _t( 0.0 ), _num_scan_angles( 128 ), _num_scan_distances( 256 ) {

}

Sim::
~Sim() {

}
void
Sim::
step( const double& dt ){
	Eigen::Vector3d uhat( _u );
	
	double b0 = _alpha1 * pow( _u( 0 ), 2 ) + _alpha2 * pow( _u( 1 ), 2 );
	double b1 = _alpha3 * pow( _u( 0 ), 2 ) + _alpha4 * pow( _u( 1 ), 2 );
	double b2 = _alpha5 * pow( _u( 0 ), 2 ) + _alpha6 * pow( _u( 1 ), 2 );
	
	uhat( 0 ) = _u( 0 ) + sample( b0 );
	uhat( 1 ) = _u( 1 ) + sample( b1 );
	uhat( 2 ) = sample( b2 );
	
	Eigen::Vector3d dx( 0.0, 0.0, 0.0 );
	
	if( uhat( 1 ) != 0.0 ){
		dx( 0 ) = - ( uhat ( 0 ) / uhat ( 1 ) ) * sin( _x( 2 ) ) + ( uhat( 0 ) / uhat( 1 ) ) * sin( _x( 2 ) + uhat ( 1 ) * dt );
		dx( 1 ) = ( uhat ( 0 ) / uhat ( 1 ) ) * cos( _x( 2 ) ) - ( uhat( 0 ) / uhat( 1 ) ) * cos( _x( 2 ) + uhat ( 1 ) * dt );
		dx( 2 ) = uhat( 1 ) * dt;
	} else {
		cout << "in the else" <<endl;
		dx( 0 ) = uhat( 0 ) * cos( _x( 2 ) ) * dt;
		dx( 1 ) = uhat( 0 ) * sin( _x( 2 ) ) * dt;
		cout << "dx( 1 ) = " << dx(1) << endl;
		dx( 2 ) = uhat( 1 ) * dt;
	}
	
	_x += dx;
	_t += dt;

	cout << "_u[3]:{" << _u( 0 ) << "," << _u( 1 ) << "," << _u( 2 ) << "}" << endl;
	cout << "uhat[3]:{" << uhat( 0 ) << "," << uhat( 1 ) << "," << uhat( 2 ) << "}" << endl;
	cout << "_x[3]:{" << _x( 0 ) << "," << _x( 1 ) << "," << _x( 2 ) << "}" << endl;
	cout << "_t:" << _t << endl << endl;
	return;
}

void
Sim::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
	_u( 0 ) = msg->linear.x;
	_u( 1 ) = msg->angular.z;
	return;
}

void
Sim::
handle_landmarks( const perception::Landmarks::ConstPtr& msg ){
	_landmarks = *msg;
	return;
}

void
Sim::
handle_observations(const perception::Observations::ConstPtr& msg ){
	_observations = *msg;
	return;
}

void
Sim::
handle_odometry(const nav_msgs::Odometry::ConstPtr& msg ){
	_odometry = *msg;
	return;
}

void
Sim::
observations_determine( void ){
	_observations.observations.clear();
	
	double range_radius = 3.0;
	double heading_range = M_PI / 2;
	
	for( unsigned int i = 0; i < _landmarks.landmarks.size(); i++ ){
		perception::Observation o;
		
		double x = _x( 0 );
		double y = _x( 1 );
		double heading = _x( 2 );
		
		double tempx = cos( heading ) * (_landmarks.landmarks[i].pos.x - x ) + sin( heading ) * (_landmarks.landmarks[i].pos.y - y );
		double tempy = -sin( heading ) * (_landmarks.landmarks[i].pos.x - x ) + cos( heading ) * (_landmarks.landmarks[i].pos.y - y );
		
		double angle = atan2( tempy, tempx );
		
		o.range = sqrt( pow( tempx, 2 ) + pow( tempy, 2 ) );
		o.bearing = angle;
		o.signature = _landmarks.landmarks[ i ].signature;
		
		if( o.range < range_radius && abs(o.bearing) < heading_range / 2){
			_observations.observations.push_back( o );
		}
	}
	observations_publisher.publish( _observations );
}

nav_msgs::Odometry
Sim::
odometry_msg( void )const{
	nav_msgs::Odometry msg;
	cout << " x(0) " << _x(0) << endl;
	cout << " x(1) " << _x(1) << endl;
	msg.pose.pose.position.x = _x( 0 );
	msg.pose.pose.position.y = _x( 1 );
	msg.pose.pose.position.z = 0.0;
	msg.twist.twist.linear.x = _u( 0 );
	msg.twist.twist.angular.z = _u( 1 );
	msg.pose.pose.orientation = yaw_to_quaternion( _x( 2 ) );
	return msg;
}

void
Sim::
handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg ){
	_obstacles = *msg;
	return;
}

sensor_msgs::LaserScan
Sim::
scan_msg( void ) const {
	sensor_msgs::LaserScan msg;
	msg.angle_min = -M_PI/2.0;
	msg.angle_max = M_PI/2.0;
	msg.angle_increment = ( msg.angle_max - msg.angle_min ) / ( double )( _num_scan_angles - 1 );
	msg.range_min = 0.45;
	msg.range_max = 10.0;
	// check to see if inside obstacle
	for ( unsigned int i = 0; i < _obstacles.points.size(); i++ ){
		double distance_to_obstacle = ( Eigen::Vector2d( _x( 0 ), _x( 1 ) ) - Eigen::Vector2d( _obstacles.points[ i ].x, _obstacles.points[ i ].y ) ).norm();
		if ( distance_to_obstacle < _obstacles.points[ i ].z ){
			return msg;
		}
	}
	double range_increment = msg.range_max / ( double )( _num_scan_distances - 1 );
	Eigen::Vector2d robot_position( _x( 0 ), _x( 1 ) );
	// simulate the scans by checking for intersection with obstacles
	for ( unsigned int i = 0; i < _num_scan_angles; i++ ){
		double angle = msg.angle_min + ( double )( i ) * msg.angle_increment;
		double min_range = msg.range_max;
		for ( unsigned int j = 0; j < _num_scan_distances; j++ ){
			double range = ( double )( j ) * range_increment;
			Eigen::Vector2d scanpoint = robot_position + Eigen::Vector2d( range * cos( _x( 2 ) + angle ), range * sin( _x( 2 ) + angle ) );
			for ( unsigned int k = 0; k < _obstacles.points.size(); k++ ){
				double distance_to_obstacle = ( scanpoint - Eigen::Vector2d( _obstacles.points[ k ].x, _obstacles.points[ k ].y ) ).norm();
				if( ( distance_to_obstacle < _obstacles.points[ k ].z ) && ( range < min_range ) ){
					min_range = range;
				}
			}
		}
		if ( min_range > msg.range_min ){
			msg.ranges.push_back( std::min( min_range + sample( 0.001 ), ( double )( msg.range_max ) ) );
		} else {
			msg.ranges.push_back( 0.0 );
		}
	}
	return msg;
}
