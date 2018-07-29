#include "geometry_msgs/Pose2D.h"
#include "pfc/drive_to_point.h"

using namespace std;

double
compute_distance( const geometry_msgs::Pose2D& a, const geometry_msgs::Pose2D& b ){
	return sqrt( pow( a.x - b.x, 2.0 ) + pow( a.y - b.y, 2.0 ) );
}

double
sgn( const double& arg ){
	if ( arg < 0.0 ){
		return -1.0;
	}else{
		return	1.0;
	}
}

geometry_msgs::Point
relative_position( const geometry_msgs::Pose2D& robot, const geometry_msgs::Pose2D& world ){
	geometry_msgs::Point local;
	local.x = cos( robot.theta ) * ( world.x - robot.x ) + sin( robot.theta ) * ( world.y - robot.y );
	local.y = -sin( robot.theta ) * ( world.x - robot.x ) + cos( robot.theta ) * ( world.y - robot.y );
	return local;
}

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion;
}

double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

geometry_msgs::PoseStamped
pose2d_to_pose_stamped( const geometry_msgs::Pose2D& pose ){
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose.position.x = pose.x;
	pose_stamped.pose.position.y = pose.y;
	pose_stamped.pose.orientation = yaw_to_quaternion( pose.theta );
	return pose_stamped;
}

geometry_msgs::Pose2D
pose_to_pose2d( const geometry_msgs::Pose& pose ){
	geometry_msgs::Pose2D pose2d;
	pose2d.x = pose.position.x;
	pose2d.y = pose.position.y;
	pose2d.theta = quaternion_to_yaw( pose.orientation );
	return pose2d;
}

nav_msgs::Path
generate_projection( const geometry_msgs::Pose2D& robot, const geometry_msgs::PoseStamped& goal ){
	nav_msgs::Path path;
	path.poses.push_back( pose2d_to_pose_stamped( robot ) );
	path.poses.push_back( goal );
	return path;
}

DriveToPoint::
DriveToPoint() : _estimated_odometry(),
				path(),
				command(),
				projection(),
				lookahead(),
				max_speed( 0.2 ),
				path_index( 0 ),
				distance_threshold( 0.1 ),
				angle_threshold( M_PI/8.0 ) {
}

DriveToPoint::
 ~DriveToPoint() {}

void
DriveToPoint::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	_estimated_odometry = *msg;
	cout << "estimated odom : " << endl << _estimated_odometry << endl;
	return;
}

void
DriveToPoint::
handlePath( const nav_msgs::Path::ConstPtr& msg ){
	path = *msg;
	path_index = 0;
	cout << "got path with " << path.poses.size() << " poses" << endl;
	return;
}

void
DriveToPoint::
updatePathIndex( void ){
	if( !path.poses.empty() ){
		geometry_msgs::Pose2D robot_pose = pose_to_pose2d( _estimated_odometry.pose.pose );
		double distance_to_path_index_pose = compute_distance( robot_pose, pose_to_pose2d( path.poses[ path_index ].pose ) );
		while( distance_to_path_index_pose < distance_threshold ){
			path_index++;
			if( path_index == path.poses.size() ){
				path.poses.clear();
				path_index = 0;
				return;
			}else{
				distance_to_path_index_pose = compute_distance( robot_pose, pose_to_pose2d( path.poses[ path_index ].pose ) );
			}
		}
		lookahead = path.poses[ path_index ].pose.position;
	}
	return;
}

void
DriveToPoint::
updateCommand( void ){
	if( !path.poses.empty() ){
		geometry_msgs::Pose2D robot_pose = pose_to_pose2d( _estimated_odometry.pose.pose );
		geometry_msgs::Point relative_lookahead = relative_position( robot_pose, pose_to_pose2d( path.poses[ path_index ].pose ) );
		cout << "turning" << endl;
		double angle_to_goal = atan2( relative_lookahead.y, relative_lookahead.x );
		if( fabs( angle_to_goal ) < angle_threshold ){
			command.linear.x = command.linear.x + 0.1;
			if( command.linear.x > max_speed ){
				command.linear.x = max_speed;
			}
			command.angular.z = 0.0;
		} else {
			// turn towards path
			command.linear.x = 0.0;
			command.angular.z = 0.5 * sgn( angle_to_goal );
		}
		projection = generate_projection( robot_pose, path.poses[ path_index ] );
	} else {
		command.linear.x = 0.0;
		command.angular.z = 0.0;
	}
	return;
}
ostream&
operator<<( ostream& out, const DriveToPoint& other ){
	return out;
}
