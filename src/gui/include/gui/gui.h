#ifndef GUI_H
#define GUI_H

#include <iostream> 
#include <map>
#include "ros/ros.h"
#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QTimer>
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "sensor_msgs/LaserScan.h"
#include "perception/Landmarks.h"
#include "perception/Observations.h"
//#include "mapper/mapper.h"

class GUI: public QGLWidget {
	Q_OBJECT
	public: 
		GUI( QWidget * parent = NULL );
		virtual ~GUI();
		
		void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg );
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleGoal( const geometry_msgs::Pose::ConstPtr& msg );
		void handlePath( const nav_msgs::Path::ConstPtr& msg );
		void handleLookahead( const geometry_msgs::Point::ConstPtr& msg );
		void handleProjection( const nav_msgs::Path::ConstPtr& msg );
		void handleLandmarks( const perception::Landmarks::ConstPtr& msg );
		void handleObservedLandmarks( const perception::Landmarks::ConstPtr& msg ) ;
		void handleObservations( const perception::Observations::ConstPtr& msg );
		void handleMap( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
		
		
	protected slots:
		void timer_callback(void);

	protected:
		virtual void initializeGL();
		virtual void resizeGL( int width, int height );
		virtual void paintGL();
		void drawCoordinateSystem( void );
		void drawGrid(); 
		void drawPoint( const geometry_msgs::Point& point, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& size = 1.0 );
		void drawLaserScan( const sensor_msgs::LaserScan& laserscan, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0 );
		void drawRobot( const geometry_msgs::Pose& pose, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& radius = 0.1225 );
		void drawRobotSensorHorizon( const geometry_msgs::Pose& pose, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& minAngle = -M_PI/4.0, const double& maxAngle = M_PI/4.0, const double& maxRange = 5.0 );
		void drawPath( const nav_msgs::Path& path, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& width = 1.0 );
		void drawLandmarks( const perception::Landmarks& polygon, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& size = 1.0 );
		void drawObservations( const perception::Observations& polygon, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& size = 1.0 );
		void drawMap( const map_msgs::OccupancyGridUpdate& map, const double& r, const double& g, const double& b );
		virtual void keyPressEvent(QKeyEvent * event);

		QTimer _timer;

		double _zoom;
		std::pair< double, double> _center;

		sensor_msgs::LaserScan _laserscan;
		nav_msgs::Odometry _odom;
		nav_msgs::Odometry _estimated_odom;
		map_msgs::OccupancyGridUpdate _map;
		geometry_msgs::Pose _goal;
		geometry_msgs::Point _lookahead;
		nav_msgs::Path _path;
		nav_msgs::Path _projection;
		perception::Landmarks _landmarks;
		perception::Observations _observations;
		std::map<int, geometry_msgs::Point > _observed_landmarks_map;
};
#endif /*GUI_H*/
