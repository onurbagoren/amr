using namespace std;


#include <iostream>

#include "planner/planner_xy.h"
#include <stdio.h>
#include <algorithm>

PlannerXY::PlannerXY() {
	
}

PlannerXY::~PlannerXY() {

}

bool PlannerXY::contains(vector<Node*> v, Node* n){
	return (find(v.begin(), v.end(), n) - v.begin()) > 0;
}

bool compareNodes(Node* n1, Node* n2) { return (n1->f < n2->f); }

bool 
PlannerXY::search( const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const double& w ){

	openList.clear();
	closedList.clear();
	path.clear();
	optimal_path.poses.clear();

	startX = round( start.position.x * 4 ) / 4;
	startY = round( start.position.y * 4 ) / 4;
	goalX = round( goal.position.x * 4 ) / 4;
	goalY = round( goal.position.y * 4 ) / 4;
	
	//cout << "start x = " << startX << " y = " << startY << endl;
	

	Node init; //initial node
	init.g = 0; //g = 0
	init.h = hypot(goalX - startX, goalY-startY);
	init.f = init.g + w * init.h; // f = g + w * h;
	init.id_x = 0; //setting id = (0,0)
	init.id_y = 0;
	init.x = startX;
	init.y = startY;
	init.pred = 0;
	
	openList.push_back(&init); //Push into open List

	//int counter = 0;

	while( !openList.empty() /*&& counter <= 25*/ ){
	
		//cout << "counter : " << counter << endl;	
			
		//counter = counter + 1;	
		
		sort(openList.begin(), openList.end(), compareNodes); //Sort according to f value
		
		Node* current = openList.at(0); //get the smallest f valued node
		
		//cout << endl <<  "Current Node, x : " << current->x << ", y : " << current->y << ", f : " << current ->f << endl;
		
		if(current->x == goalX && current->y == goalY){ //if smallest f value is goal, return true
			closedList.push_back(current);
			cout << "closed list size: " << closedList.size() << endl << "done!" << endl;
			return true;
		}
		closedList.push_back(current); //put lowest f valued node into closed list

		//Neighbouring nodes to Node* current
		Node* n1 = new Node();
		n1->x = current->x + 0.25;
		n1->y = current->y;
		n1->pred = current;
		n1->g = current->g + 0.25;
		n1->h = hypot(goalX - n1->x, goalY - n1->y);
		n1->f = n1->g + w * n1->h;
		n1->id_x = current->id_x + 1;
		n1->id_y = current->id_y;

		Node* n2 = new Node();
		n2->x = current->x;
		n2->y = current->y + 0.25;
		n2->pred = current;
		n2->g = current->g + 0.25;
		n2->h = hypot(goalX - n2->x, goalY - n2->y);
		n2->f = n2->g + w * n2->h;
		n2->id_x = current->id_x + 1;
		n2->id_y = current->id_y;

		Node* n3 = new Node();
		n3->x = current->x + 0.25;
		n3->y = current->y + 0.25;
		n3->pred = current;
		n3->g = current->g + hypot(0.25, 0.25);
		n3->h = hypot(goalX - n3->x, goalY - n3->y);
		n3->f = n3->g + w * n3->h;
		n3->id_x = current->id_x + 1;
		n3->id_y = current->id_y + 1;

		Node* n4 = new Node();
		n4->x = current->x - 0.25;
		n4->y = current->y;
		n4->pred = current;
		n4->g = current->g + 0.25;
		n4->h = hypot(goalX - n4->x, goalY - n4->y);
		n4->f = n4->g + w * n4->h;
		n4->id_x = current->id_x - 1;
		n4->id_y = current->id_y;

		Node* n5 = new Node();
		n5->x = current->x;
		n5->y = current->y - 0.25;
		n5->pred = current;
		n5->g = current->g + 0.25;
		n5->h = hypot(goalX - n5->x, goalY - n5->y);
		n5->f = n5->g + w * n5->h;
		n5->id_x = current->id_x;
		n5->id_y = current->id_y - 1;
		
		Node* n6 = new Node();
		n6->x = current->x - 0.25;
		n6->y = current->y - 0.25;
		n6->pred = current;
		n6->g = current->g + hypot(0.25, 0.25);
		n6->h = hypot(goalX - n6->x, goalY - n6->y);
		n6->f = n6->g + w * n6->h;
		n6->id_x = current->id_x - 1;
		n6->id_y = current->id_y - 1;

		Node* n7 = new Node();
		n7->x = current->x + 0.25;
		n7->y = current->y - 0.25;
		n7->pred = current;
		n7->g = current->g + hypot(0.25, 0.25);
		n7->h = hypot(goalX - n7->x, goalY - n7->y);
		n7->f = n7->g + w * n7->h;
		n7->id_x = current->id_x + 1;
		n7->id_y = current->id_y - 1;
		
		Node* n8 = new Node();
		n8->x = current->x - 0.25;
		n8->y = current->y + 0.25;
		n8->pred = current;
		n8->g = current->g + hypot(0.25, 0.25);
		n8->h = hypot(goalX - n8->x, goalY - n8->y);
		n8->f = n8->g + w * n8->h;
		n8->id_x = current->id_x - 1;
		n8->id_y = current->id_y + 1;

		//Insert to the children vector of Node* current
		
		sort(current->children.begin(), current->children.end(), compareNodes);

		if( check_map( n1->x, n1->y ) ){
			current->children.push_back( n1 );
		}
		if( check_map( n2->x, n2->y ) ){
			current->children.push_back( n2 );
		}
		if( check_map( n3->x, n3->y ) ){
			current->children.push_back(n3);
		}
		if( check_map( n4->x, n4->y ) ){
			current->children.push_back(n4);
		}
		if( check_map( n5->x, n5->y ) ){
			current->children.push_back(n5);
		}
		if( check_map( n6->x, n6->y ) ){
			current->children.push_back(n6);
		}
		if( check_map( n7->x, n7->y ) ){
			current->children.push_back(n7);
		}
		if( check_map( n8->x, n8->y ) ){
			current->children.push_back(n8);
		}
		
		for( int i = 0; i < current->children.size(); i++ ){
			openList.push_back( current->children.at( i ) );
		}
		
		for( int i = 0; i < openList.size(); i++ ){
			for( int j = i + 1; j < openList.size(); j++){
				if( openList.at( i )->x == openList.at( j )->x && openList.at( i )->y == openList.at( j )->y ){
					if( openList.at( i )->f > openList.at( j )->f ){
						openList.erase( openList.begin() + i );
					}else{
						openList.erase( openList.begin() + j );
					}
				}
			}
			
			for( int j = 0; j < closedList.size(); j++ ){
				if( closedList.at( j ) == openList.at( i ) ){
					openList.erase( openList.begin() + i );
				}
			}
		}
		
	}

	return false;

};

nav_msgs::Path
PlannerXY::generate_path( void ){
	    
	path.clear();
	Node* end = closedList.at( closedList.size() - 1 );
	
	Node* beginning = closedList.at( 0 );
	
	Node* prev = end->pred;
	
	path.push_back( end );
	
	while( prev->x != beginning->x && prev->y != beginning->y ){
		//cout<< "adding to path" << endl;
		path.push_back( prev );
		prev = prev->pred;
	}
	
	//path.push_back( beginning );
	
	reverse(path.begin(),path.end()); //reversing to get the true path
	
	nav_msgs::Path optimal_path;
	geometry_msgs::PoseStamped pose_stamped;
	
	for( int i = 0; i < path.size(); i++ ){
		//cout << "path[ " << i << " ], x: " << path.at( i )->x << ", y: " << path.at( i )->y << endl;
		pose_stamped.pose.position.x = path.at(i)->x;
		pose_stamped.pose.position.y = path.at(i)->y;
		optimal_path.poses.push_back( pose_stamped );
	}
	
	return optimal_path;
};



std_msgs::UInt32
PlannerXY::open_list_size( void ){
	std_msgs::UInt32 closedListS;
	std_msgs::UInt32 openListS;
	closedListS.data = closedList.size();
	openListS.data = openList.size(); //getting the size and inputting to the data of UInt32
	//cout << "open list size : " << openListS.data << endl;
	//openlistsize_publisher.publish( openListS );
	return openListS;
};

void 
PlannerXY::handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	_estimated_odometry = *msg;
	return;
}

void
PlannerXY::handle_goal( const geometry_msgs::Pose::ConstPtr& msg ){
	
	geometry_msgs::Pose goal = *msg;
	search( _estimated_odometry.pose.pose, goal, 1);
	cout << "starting pose : " << _estimated_odometry.pose.pose.position.x << ", " << _estimated_odometry.pose.pose.position.y << endl;
	cout << "goal pose : " << goal.position.x << ", " << goal.position.y << endl;
	optimal_path = generate_path();
	path_publisher.publish( optimal_path );
	cout << "published path" << endl;
	optimal_path.poses.clear();
	return;

}

bool
PlannerXY::
check_map( const double& x, const double& y ){
	return mapper.checkMap( x , y , 0.25, 1.0 );
}
