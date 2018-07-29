#ifndef MAPPER_H
#define MAPPER_H
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "map_msgs/OccupancyGridUpdate.h"

class Mapper {

	public:
	
		Mapper( const double& discretization = 0.1, const unsigned int& numRows = 801, const unsigned int& numCols = 801 );
		virtual ~Mapper();
	
		void handleOccupancyGridUpdate( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
		void handleOdometry( const nav_msgs::Odometry::ConstPtr& msg );
		void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg );
		bool checkMap( const double& x, const double& y, const double& radius, const double& threshold );
		void update( void );
		
		map_msgs::OccupancyGridUpdate _map;
		
		inline map_msgs::OccupancyGridUpdate& map_msg( void ){ return _map; };
		
	protected:
	
		double _discretization;
		std::vector< double > _xs;
		std::vector< double > _ys;
		double _l0;
		double _locc;
		double _lfree;
		nav_msgs::Odometry _odometry;
		std::vector< sensor_msgs::LaserScan > _scans;
		
};

std::ostream& operator<<( std::ostream& out, const Mapper& other );

#endif /* MAPPER_H */
