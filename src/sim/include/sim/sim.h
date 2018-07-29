#ifndef SIM_H
#define SIM_H
#include <Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/LaserScan.h"
#include "perception/Landmarks.h"
#include "perception/Observations.h"
#include "perception/Observation.h"

class Sim {
	public:
	  	Sim();
		virtual ~Sim();

		void step( const double& dt ); 
		void update_observed_landmarks( void );
		void update_observations( void ); 
		void handle_command( const geometry_msgs::Twist::ConstPtr& msg );
		void handle_landmarks( const perception::Landmarks::ConstPtr& msg );
		void handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg ); 
		nav_msgs::Odometry odometry_msg( void )const;
		sensor_msgs::LaserScan scan_msg( void )const;
		perception::Landmarks& landmarks( void ){ return _landmarks; };
		perception::Landmarks& observed_landmarks( void ){ return _observed_landmarks; };
		perception::Observations& observations( void ){ return _observations; };
		geometry_msgs::Polygon& obstacles( void ){ return _obstacles; };
		void handle_observations(const perception::Observations::ConstPtr& msg );
		void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg );
		void observations_determine( void );
		ros::Publisher observations_publisher;
		

	protected:
		Eigen::Vector3d _x;
		Eigen::Vector3d _u;
		double _alpha1;
		double _alpha2;
		double _alpha3;
		double _alpha4;
		double _alpha5;
		double _alpha6;
		double _t;
		unsigned _num_scan_angles;
		unsigned _num_scan_distances;
		geometry_msgs::Polygon _obstacles;
		double _observationMaxRange;	
		double _observationMaxAngle;
		perception::Landmarks _landmarks;
		perception::Landmarks _observed_landmarks;
		perception::Observations _observations;
		nav_msgs::Odometry _odometry;
};

#endif /* SIM_H */
