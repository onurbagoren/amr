#include <iostream>
#include <GL/gl.h>
#include<GL/glu.h>
#include "gui/gui.h"
using namespace std;
double 
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0*( quaternion.w*quaternion.z + quaternion.x*quaternion.y ), 1.0 - 2.0*	  ( quaternion.y*quaternion.y + quaternion.z*quaternion.z ) );
}

GUI::
GUI( QWidget * parent ) : QGLWidget( parent ),
									_timer(),
									_zoom( 5.0 ),
									_center( 0.0, 0.0 ),
									_laserscan(),
									_odom(),
									_goal(),
									_path() {
	setMinimumSize( 600, 600 );
	setFocusPolicy(Qt::StrongFocus);

	connect( &_timer, SIGNAL( timeout() ),this, SLOT( timer_callback() ) );
	_timer.start( 10 );
}

GUI::
~GUI() {
}

void
GUI::
handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg ){
	cout << "in handleLaserscan" << endl;
	_laserscan = *msg;
	updateGL();
	return;
}

void
GUI::
handleMap( const map_msgs::OccupancyGridUpdate::ConstPtr& msg ){
	_map = *msg;
	return;
}

void
GUI::
handleOdom(const nav_msgs::Odometry::ConstPtr& msg ){
	cout << "in handleOdom" << endl;
	_odom = *msg;
	updateGL();
	return;
}

void
GUI::
handleObservations(const perception::Observations::ConstPtr& msg ){
	_observations = *msg;
	return;
}

void
GUI::
handleGoal(const geometry_msgs::Pose::ConstPtr& msg ){
	cout << "in handleGoal" << endl;
	_goal = *msg;
	updateGL();
	return;
}

void
GUI::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	cout << "in handleEstimatedOdom" << endl;
	_estimated_odom = *msg;
	updateGL();
	return;
}

void
GUI::
handleLandmarks( const perception::Landmarks::ConstPtr& msg ){
	_landmarks = *msg;
	return;
}

void
GUI::
handlePath( const nav_msgs::Path::ConstPtr& msg ){
	cout << "in handlePath" << endl;
	_path = *msg;
	updateGL();
	return;
}

void
GUI::
timer_callback( void ){
	ros::spinOnce();
	return;
}

void
GUI::
initializeGL(){
	glClearColor( 1.0, 1.0, 1.0, 1.0 );
	glEnable( GL_LINE_SMOOTH );
	glEnable( GL_BLEND );
	return;
}

void
GUI::
resizeGL( int width, int height ){
	glViewport( 0, 0, width, height );
	return;
}

void
GUI::
drawCoordinateSystem(void){
	glBegin( GL_LINES );
	glColor4f( 1.0, 0.0, 0.0, 1.0 );
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 1.0, 0.0, 0.0 );
	glColor4f( 0.0, 1.0, 0.0, 1.0 );
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 0.0, 1.0, 0.0 );
	glColor4f( 0.0, 0.0, 1.0, 1.0 );
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 0.0, 0.0, 1.0 );
	glEnd();
	return;
}

void
GUI::
drawGrid(void){
	glColor4f( 0.8, 0.8, 0.8, 1.0 );
	glLineWidth( 2.0 );
	glBegin( GL_LINES );
	for( int i = -10; i <= 10; i++ ){
		glVertex3f( -10.0, (double)( i ), 0.0 );
		glVertex3f( 10.0, (double)( i ), 0.0 );
		glVertex3f( (double)( i ), -10.0, 0.0 );
		glVertex3f( (double)( i ), 10.0, 0.0 );
	}
	glEnd();
	glLineWidth( 1.0 );
}

void
GUI::
drawLaserScan(const sensor_msgs::LaserScan& laserscan,
				const double& red,
				const double& green,
				const double& blue ){
	glPushMatrix();
	glTranslated( _estimated_odom.pose.pose.position.x, _estimated_odom.pose. pose.position.y, 0.0 );
	glRotated( quaternion_to_yaw( _estimated_odom.pose.pose.orientation ) * 180.0 / M_PI, 0.0, 0.0, 1.0 );
	glColor4f( 0.0, 1.0, 0.2, 1.0 );
	glLineWidth( 2.0 );
	glBegin( GL_LINES );
	for( unsigned int i = 0; i < _laserscan.ranges.size(); i++ ){
		double angle = _laserscan.angle_min + ( double )( i ) * _laserscan. angle_increment;
		glVertex3f( 0.0, 0.0, 0.0 );
		glVertex3f( _laserscan.ranges[ i ] * cos( angle ), _laserscan.ranges[ i ] * sin( angle ), 0.0 );
	}
	glEnd(); 
	glLineWidth( 1.0 ); 
	glPopMatrix();
	return;
}

void
GUI::
drawRobot(  const geometry_msgs::Pose& pose, const double& red, const double& green, const double& blue, const double& radius ){
	glPushMatrix();
	glTranslated( pose.position.x, pose.position.y, 0.0 );
	glRotated( quaternion_to_yaw( pose.orientation ) * 180.0 / M_PI, 0.0, 0.0, 1.0 );
	unsigned int discretization = 33;
	glColor4f( red, blue, green, 1.0 );
	glLineWidth( 5.0 );
	glBegin( GL_LINE_STRIP );
	for( unsigned int i = 0; i < discretization; i++ ){
		double angle = 2.0 * M_PI * ( double )( i ) / ( double )( discretization - 1 );
		glVertex3f( radius*cos( angle ), radius*sin( angle ), 0.0 );
	}
	glEnd();
	glBegin( GL_LINES );
	glVertex3f( radius, 0.0, 0.0 );
	glVertex3f( -radius, 0.0, 0.0 );
	glEnd();
	glBegin( GL_TRIANGLES );
	glVertex3f( radius, 0.0, 0.0 );
	glVertex3f( radius / 4.0, radius / 2.0, 0.0 );
	glVertex3f( radius / 4.0, -radius / 2.0, 0.0 );
	glEnd();
	glLineWidth( 1.0 );
	glPopMatrix();
	return;
}

void
GUI::
keyPressEvent(QKeyEvent * event){
	if( event->matches( QKeySequence::Copy ) ){
	  close();
	  return;
	  } else {
		switch (event->key()) {
		case Qt::Key_Left:
			_center.first -= 0.5;
			break;
		case Qt::Key_Right:
			_center.first += 0.5;
			break;
		case Qt::Key_Down:
			_center.second -= 0.5;
			break;
		case Qt::Key_Up:
			_center.second += 0.5;
			break;
		case Qt::Key_I:
			if( _zoom > 0.5 ){
				_zoom -= 0.5;
				}
			break;
		case
			Qt::Key_O:
			_zoom += 0.5;
		break;
		default:
		cout << "could not handle key" << event->key() << endl;
		break;
		}
		updateGL();
	}	
	return;
}

void
GUI::
drawMap( const map_msgs::OccupancyGridUpdate& map, const double& r, const double& g, const double& b ){
	double discretization = 0.1;
	double half_discretization = discretization / 2.0;
	double min_x = -( double )( map.width - 1 ) * half_discretization; 
	double min_y = -( double )( map.height - 1 ) * half_discretization;
	glPushMatrix();
	glBegin( GL_QUADS );
	for( unsigned int i = 0; i < map.width; i++ ){
		double x = min_x + ( double )( i ) * discretization;
		for( unsigned int j = 0; j < map.height; j++ ){
			double y = min_y + ( double )( j ) * discretization;
			double occ = 1.0 - ( 1.0 / ( 1.0 + exp( ( double )( map.data[ i * map. height + j ] ) * 0.05 ) ) );
			glColor4f( ( 1.0 - occ ) * r, ( 1.0 - occ ) * g, ( 1.0 - occ ) * b, 1.0 );
			glVertex3f( x - half_discretization, y - half_discretization, 0.0 );
			glVertex3f( x + half_discretization, y - half_discretization, 0.0 );
			glVertex3f( x + half_discretization, y + half_discretization, 0.0 );
			glVertex3f( x - half_discretization, y + half_discretization, 0.0 );
		}
	}
	glEnd();
	glPopMatrix();
	return;
}

void
GUI::
handleLookahead(const geometry_msgs::Point::ConstPtr& msg ){
	_lookahead =  *msg;
}

void
GUI::
handleProjection(const nav_msgs::Path::ConstPtr& msg ){
	_projection = *msg;
	return;
}

void
GUI::
drawLandmarks( const perception::Landmarks& _landmarks,  const double& red, const double& green, const double& blue, const double& size){
	for( unsigned int i = 0; i < _landmarks.landmarks.size(); i++ ){
		drawPoint( _landmarks.landmarks[i].pos, red, green, blue, size);
	}
}

void
GUI::
drawObservations( const perception::Observations& _observations,  const double& red, const double& green, const double& blue, const double& size ){
	for( unsigned int i = 0; i < _observations.observations.size(); i++ ){
		for( unsigned int j = 0; j < _landmarks.landmarks.size(); j++){
			if( _landmarks.landmarks[ j ].signature == _observations.observations[ i ].signature ){
				geometry_msgs::Point point;
				point.x = _odom.pose.pose.position.x + _observations.observations[i].range * cos( _observations.observations[i].bearing + quaternion_to_yaw( _odom.pose.pose.orientation ) );
				point.y = _odom.pose.pose.position.y + _observations.observations[i].range * sin ( _observations.observations[i].bearing + quaternion_to_yaw( _odom.pose.pose.orientation ) );
				drawPoint( point, 0.0, 0.0, 0.0, 7.5);
			}
		}
	}
}

void
GUI::
paintGL(){
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double ratio = (double)( size().width() ) / (double)( size().height() );
	gluOrtho2D( -_zoom * ratio + _center.first, _zoom * ratio + _center.first, - _zoom + _center.second, _zoom + _center.second );
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	drawGrid();
	drawCoordinateSystem();
	drawMap( _map, 1.0, 1.0, 1.0 );
	drawLaserScan( _laserscan, 0.0, 0.0, 1.0 );
	drawRobot( _odom.pose.pose, 0.0, 0.0, 0.0, 0.1225 );
	drawRobot( _estimated_odom.pose.pose, 0.5, 1.0, 1.0, 0.1225 * 2 );
	//drawRobotSensorHorizon( _odom.pose.pose, 0.2, 0.2, 0.2, -M_PI/4.0, M_PI/4.0, 5.0 );
	drawRobot( _goal, 0.0, 1.0, 0.0, 0.1225 );
	drawPath( _path, 1.0, 0.0, 0.0, 1.0 );
	drawPath( _projection, 1.0, 0.0, 1.0, 1.0 );
	drawPoint( _lookahead, 0.0, 0.3, 1.0, 5.0 );
	drawLandmarks( _landmarks, 1.0, 0.0, 0.0, 15.0 );
	drawObservations( _observations, 0.5, 0.5, 0.0, 15.0 );
	return;
}

void
GUI::
drawPoint( const geometry_msgs::Point& point, const double& red, const double& green, const double& blue, const double& size ){
	glPointSize( size );
	glBegin( GL_POINTS );
	glColor4f( red, green, blue, 1.0 );
	glVertex3f( point.x, point.y, point.z );
	glEnd();
	glPointSize( 1 );
	return;
}
	
void
GUI::
drawPath(const nav_msgs::Path& path, const double& red, const double& green, const double& blue, const double& width ){
	glLineWidth( width );
	glColor4f( red, green, blue, 1.0 );
	glBegin( GL_LINE_STRIP );
	for( unsigned int i = 0; i < path.poses.size(); i++ ){
		glVertex3f( path.poses[ i ].pose.position.x, path.poses[ i ].pose.position.y, 0.1);
	}
	glEnd();
	glLineWidth( 1.0 );
	return;
}

void
GUI::
drawRobotSensorHorizon( const geometry_msgs::Pose& pose, const double& red, const double& green, const double& blue, const double& minAngle, const double& maxAngle, const double& maxRange ){
	
	
	
}


	
	
