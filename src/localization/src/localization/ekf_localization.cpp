#include "localization/ekf_localization.h"
#include "geometry_msgs/Point.h"
#include "perception/Landmarks.h"
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "perception/Observations.h"


using namespace std;

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion;
}

EKF_Localization::
EKF_Localization( const Eigen::VectorXd& alpha, const Eigen::MatrixXd& q ) : _u(),
															  _landmarks(),
														 	  _z(),
															  _mu( Eigen::VectorXd::Zero( 3 ) ),
				    	 									  _sigma( Eigen::MatrixXd::Zero(3, 3 ) ),
				 											  _alpha( alpha ),
															  _q( q ) {
}

EKF_Localization::
~EKF_Localization() {
}

void
EKF_Localization::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
	_u = *msg;
	return;
}

void
EKF_Localization::
handle_odometry( const nav_msgs::Odometry::ConstPtr& msg ){
	//cout << " handling odom " << _u << endl;
	_u = msg->twist.twist;
	return;
}

void
EKF_Localization::
handle_landmarks( const perception::Landmarks::ConstPtr& msg ){
	for ( unsigned int i = 0; i < msg->landmarks.size(); i++ ){
		map< int, geometry_msgs::Point >::iterator it_landmark = _landmarks.find(msg->landmarks[ i ].signature );
		if ( it_landmark != _landmarks.end() ){
			it_landmark->second = msg->landmarks[ i ].pos;
		}else{
			_landmarks.insert( pair< int, geometry_msgs::Point >( msg->landmarks[ i ].signature, msg->landmarks[ i ].pos ) );
		}
	}
	return;
}

void
EKF_Localization::
handle_observations( const perception::Observations::ConstPtr& msg ){
	_z = *msg;
	return;
}

void
EKF_Localization::
step( const double& dt ){

	double _vt = _u.linear.x;
	double _wt = _u.angular.z;

	//Algorithm from Probabilistic Robotics - EKF Localization
	
	if( _wt == 0 ) {
		_wt = 0.0001;
	}
	
	Eigen::MatrixXd Gt = Eigen::MatrixXd::Zero( 3, 3 );
	Gt( 0, 0 ) = 1.0;
	Gt( 0, 2 ) = -( _vt / _wt ) * cos( _mu ( 2 ) ) - ( _vt / _wt ) * cos( _mu( 2 ) + _wt * dt);
	Gt( 1, 1 ) = 1.0;
	Gt( 1, 2 ) = -( _vt / _wt ) * sin( _mu ( 2 ) ) - ( _vt / _wt ) * sin( _mu( 2 ) + _wt * dt);
	Gt( 2, 2 ) = 1.0;
	
	Eigen::MatrixXd Vt = Eigen::MatrixXd::Zero( 3, 2 );
	
	Vt( 0, 0 ) = ( -sin( _mu( 2 ) ) + sin( _mu( 2 ) + _wt * dt ) ) / _wt ;
	Vt( 0, 1 ) = ( ( _vt * ( sin( _mu( 2 ) ) + sin( _mu( 2 ) + _wt * dt ) ) ) / ( _wt * _wt ) ) + ( _vt * cos( _mu( 2 ) + _wt * dt ) * dt ) / _wt;
	Vt( 1, 0 ) = ( ( cos( _mu( 2 ) ) - cos(  _mu( 2 ) + _wt * dt) )/ _wt );
	Vt( 1, 1 ) = -( ( _vt * ( cos( _mu( 2 ) ) - cos( _mu ( 2 ) + _wt * dt ) ) ) / ( _wt * _wt ) ) + ( _vt * sin( _mu( 2 ) + _wt * dt ) * dt ) / _wt;
	Vt( 2, 1 ) = dt;
	
	Eigen::MatrixXd Mt = Eigen::MatrixXd::Zero( 2, 2 );
	Mt( 0, 0 ) = _alpha( 0 ) * pow( _vt, 2 ) + _alpha( 1 ) * pow( _wt, 2 );
	Mt( 1, 1 ) = _alpha( 2 ) * pow( _vt, 2 ) + _alpha( 3 ) * pow( _wt, 2 );
	
	Eigen::VectorXd _mu_h = Eigen::VectorXd::Zero( 3 );
	_mu_h( 0 ) = _mu( 0 ) + ( ( -_vt / _wt * sin( _mu( 2 ) ) )  + ( _vt / _wt * sin( _mu( 2 ) + _wt * dt ) ) );
	_mu_h( 1 ) = _mu( 1 ) + (( _vt / _wt * cos( _mu( 2 ) ) )  - ( _vt / _wt * cos( _mu( 2 ) + _wt * dt ) ) );
	_mu_h( 2 ) = _mu( 2 ) +  _wt * dt;
	
	Eigen::MatrixXd _sigma_h = Eigen::MatrixXd::Zero( 3, 3 );
	_sigma_h = Gt * _sigma * Gt.transpose() + Vt * Mt * Vt.transpose();
	
	Eigen::VectorXd _z_h = Eigen::VectorXd::Zero( 3 );
	Eigen::VectorXd _dz = Eigen::VectorXd::Zero( 3 );
	Eigen::MatrixXd Ht = Eigen::MatrixXd::Zero( 3, 3 );
	Eigen::MatrixXd St = Eigen::MatrixXd::Zero( 3, 3 );
	Eigen::MatrixXd Kt = Eigen::MatrixXd::Zero( 3, 3 );
	//cout << "making I" << endl;
	Eigen::MatrixXd _I = Eigen::MatrixXd::Zero( 3, 3 );
	_I( 0, 0 ) = 1.0;
	_I( 1, 1 ) = 1.0;
	_I( 2, 2 ) = 1.0;
	Eigen::VectorXd _z_new = Eigen::VectorXd::Zero( 3 );
	
	for ( unsigned int i = 0; i < _z.observations.size(); i++ ){	
		cout << "observations[ " << i << " ], range = " << _z.observations[ i ].range << ", bearing = " << _z.observations[ i ].bearing << ",signature = "<< _z.observations[ i ].signature << endl;
		cout << "for observation[ " << i  << " ], _mu_h( 0 ) : " << _mu_h( 0 ) << " _mu_h( 1 ) : " << _mu_h( 1 ) << " _mu_h( 2 ): " << _mu_h( 2 ) << endl << endl;
	
		geometry_msgs::Point curr_landmark = _landmarks[ _z.observations[ i ].signature ];
		
		cout << "current landmark x: " << curr_landmark.x - _mu_h(0) << ", y: " << curr_landmark.y - _mu_h(1) << endl;
		
		double dist_squared = pow( curr_landmark.x - _mu_h(0), 2 ) + pow( curr_landmark.y - _mu_h( 1 ), 2 );
		
		//cout << "distance = " << sqrt( dist_squared ) << endl;
		
		_z_h( 0 ) = sqrt( dist_squared );
		_z_h( 1 ) = atan2( curr_landmark.y - _mu_h(1), curr_landmark.x - _mu_h(0) ) - _mu_h( 2 );
		_z_h( 2 ) = _z.observations[ i ].signature; 
		
		cout << "denom: " << sqrt(dist_squared) << ", " << "whole expression: " << - ( curr_landmark.x - _mu_h( 0 ) ) / sqrt( dist_squared ) << endl;
		Ht( 0, 0 ) = - (double)( curr_landmark.x - _mu_h( 0 ) ) / (double)(sqrt( dist_squared ));
		Ht( 0, 1 ) = - (double)( curr_landmark.y - _mu_h( 1 ) ) / (double)sqrt( dist_squared );
		Ht( 1, 0 ) = (double)( curr_landmark.y - _mu_h( 1 ) ) / dist_squared;
		Ht( 1, 1 ) = - (double)( curr_landmark.x - _mu_h( 1 ) ) / dist_squared;
		Ht( 1, 2 ) = - 1.0;
		
		St = Ht * _sigma_h * Ht.transpose() + _q;

		Kt = _sigma_h * Ht.transpose() * St.inverse();
		
		_z_new( 0 ) = _z.observations[ i ].range;
		_z_new( 1 ) = _z.observations[ i ].bearing;
		_z_new( 2 ) = _z.observations[ i ].signature;
		
		_dz = _z_new - _z_h;
		
		_dz( 1 ) = fmod( _dz( 1 ), 2 * M_PI );
				
		_mu_h = _mu_h + Kt * _dz;
		_sigma_h = ( _I - Kt * Ht ) * _sigma_h;
	}
	
	_mu = _mu_h;
	_sigma = _sigma_h;

	// clear past observations
	_z.observations.clear();
	return;
}

nav_msgs::Odometry
EKF_Localization::
estimated_odometry( void )const{
	nav_msgs::Odometry msg;
	msg.pose.pose.position.x = _mu( 0 );
	msg.pose.pose.position.y = _mu( 1 );
	msg.pose.pose.orientation = yaw_to_quaternion( _mu( 2 ) );
	return msg;
}
