#include <Eigen/Dense>
#include "mapper/mapper.h"

using namespace std;

bool row_col_in_map( const map_msgs::OccupancyGridUpdate& map, const int& row, const int& col ){
	if ( ( row < 0 ) || ( col < 0 ) ){
		return false ;
	} else if ( ( row >= map.width ) || ( col >= map.height ) ){
		return false;
	}else{
		return true;
	}
}

double row_to_x( const map_msgs::OccupancyGridUpdate& map, const int& row, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.width ) - 1 ) / 2 + row ) );
}

double col_to_y( const map_msgs::OccupancyGridUpdate& map, const int& col, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.height ) - 1 ) / 2 + col ) );
}

int x_to_row( const map_msgs::OccupancyGridUpdate& map, const double& x, const double& discretization ){
	return round( x / discretization + ( double )( ( map.width - 1 ) / 2 ) );
}

int y_to_col( const map_msgs::OccupancyGridUpdate& map, const double& y, const double& discretization ){
	return round( y / discretization + ( double )( ( map.height - 1 ) / 2 ) );
}

double quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

int8_t
double_to_int8( const double& arg ){

	int tmp = ( int )( round( arg * 20.0 ) );

	if( tmp < -127 ){
		tmp = -127;
	}else if( tmp > 127 ){
		tmp = 127;
	}

	return ( int8_t )( tmp );
}

double
int8_to_double(const int8_t& arg ){
	return ( double )( arg ) * 0.05;
}

Mapper::Mapper( const double& discretization, const unsigned int& width, const unsigned int& height ) : _discretization( discretization ),
																				    _xs( width ),
																				    _ys( height ),
																				    _l0( log( 0.5 / ( 1.0 - 0.5 ) ) ),
																				    _locc( log( 0.9 / ( 1.0 - 0.9 ) ) ),
																				    _lfree( log( 0.1 / ( 1.0 - 0.1 ) ) ),
																				    _odometry(),
																				    _scans(),
																				    _map() {
	_map.width = width;
	for( unsigned int i = 0; i < _map.width; i++ ){
		_xs[ i ] = row_to_x( _map, i, _discretization );
	}

	_map.height = height;
	for( unsigned int i = 0; i < _map.height; i++ ){
		_ys[ i ] = col_to_y( _map, i, _discretization );
	}

	_map.data.resize( _map.width * _map.height );
	for( unsigned int i = 0; i < _map.width * _map.height; i++ ){
		_map.data[ i ] = double_to_int8( _l0 );
	}
}

Mapper::
 ~Mapper() {
}

void
Mapper::
handleOdometry( const nav_msgs::Odometry::ConstPtr& msg ){
	_odometry = *msg;
	return;
}

void
Mapper::
handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ){
	_scans.push_back( *msg );
	return;
}

void
Mapper::
handleOccupancyGridUpdate( const map_msgs::OccupancyGridUpdate::ConstPtr& msg ){
	_map = *msg;
	return;
}

void
Mapper::
update( void ){
if( !_scans.empty() ){
	double x = _odometry.pose.pose.position.x;
	double y = _odometry.pose.pose.position.y;
	double yaw = quaternion_to_yaw( _odometry.pose.pose.orientation );
	
	cout << "updating " << _scans.size() << " scans" << endl;
	
	for( unsigned int i = 0; i < _scans.size(); i++ ){
		vector< int > occupied_cells;
		vector< int > free_cells;
		// search over all ranges
		for ( unsigned int j = 0; j < _scans[ i ].ranges.size(); j++ ){
			double scan_angle = _scans[ i ].angle_min + j * _scans[ i ].angle_increment;
			// check for occupied cells
			if ( fabs( _scans[ i ].ranges[ j ] - _scans[ i ].range_max ) > 0.1 ){
				double scan_x = x + _scans[ i ].ranges[ j ] * cos( yaw + scan_angle );
				double scan_y = y + _scans[ i ].ranges[ j ] * sin( yaw + scan_angle );
				int row = x_to_row( _map, scan_x, _discretization );
				int col = y_to_col( _map, scan_y, _discretization );
				if ( row_col_in_map( _map, row, col ) ){
					int index = row * _map.height + col;
					if ( find( occupied_cells.begin(), occupied_cells.end(), index ) == occupied_cells.end() ){
						occupied_cells.push_back( index );
					}
				}
			}
			double scan_distance = 0.0;
			double scan_increment = 0.1;
			// check for free cells
			while
			( scan_distance < _scans[ i ].ranges[ j ] ){
				double scan_x = x + scan_distance * cos( yaw + scan_angle );
				double scan_y = y + scan_distance * sin( yaw + scan_angle );
				int row = x_to_row( _map, scan_x, _discretization );
				int col = y_to_col( _map, scan_y, _discretization );
				if ( row_col_in_map( _map, row, col ) ){
					int index = row * _map.height + col;
					if ( ( find( free_cells.begin(), free_cells.end(), index ) == free_cells.end() ) && ( find( occupied_cells.begin(), occupied_cells.end(), index ) == occupied_cells.end() ) ){
						free_cells.push_back( index );
					}
				}
				scan_distance += scan_increment;
			}
		}
		// updating free _map log probability
		for ( unsigned int j = 0; j < free_cells.size(); j++ ){
			_map.data[ free_cells[ j ] ] = double_to_int8( int8_to_double( _map.data[ free_cells[ j ] ] ) + _lfree - _l0 );
		}
		// updating occupied _map log probability
		for ( unsigned int j = 0; j < occupied_cells.size(); j++ ){
			_map.data[ occupied_cells[ j ] ] = double_to_int8( int8_to_double( _map.data[ occupied_cells[ j ] ] ) + _locc - _l0 );
		}
	}
}
	_scans.clear();
	return;
}

bool
Mapper::
checkMap( const double& x, const double& y, const double& radius, const double& threshold ){

	int row = x_to_row( _map, x, _discretization );
	int col = y_to_col( _map, y, _discretization );
	int offset = ceil( radius / _discretization );
	
	for( int i = -offset; i < offset; i++ ){
		double dx = row_to_x( _map, row + i, _discretization ) - x;
		for( int j = -offset; j < offset; j++ ){
			double dy = col_to_y( _map, col + j, _discretization ) - y;
			if( row_col_in_map( _map, row + i, col + j ) && ( sqrt( dx * dx + dy * dy ) < radius ) ){
				int index = ( row + i ) * _map.height + col + j;
				if( _map.data[ index ] > threshold ){
					return false;
				}
			}
		}
	}
	return true;
}

ostream& operator <<( ostream& out, const Mapper& other ){
	return out;
}
