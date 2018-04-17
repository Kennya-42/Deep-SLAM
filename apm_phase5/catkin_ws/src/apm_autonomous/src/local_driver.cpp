/**
*	pilot.cpp
*	Pioneer robot driver - this node takes a set of waypoints
*		and drives the robot through them
*	Alex Avery
*	2/6/16
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>


#define TWO_PI 6.283185f
#define THREE_PI_OVER_2 4.712388f 
// Define the desired precision multiplier. For a precision of
//   .1, multiply by 1/.1 = 10
#define PRECISION_MULT 10

#define ADVANCE_TARGET_RANGE 1.5
using namespace std;

// Create current_pos and orient as a globals so they can be set in the callback
geometry_msgs::Point current_pos; // See hw3/include/hw3/Point.h
geometry_msgs::Quaternion current_orient;
bool got_current_position = false;
double current_heading = 0;

/*
*	Callback function for receiving odom data
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	// Get positional odom data
	current_pos.x = msg->pose.pose.position.x;
	current_pos.y = msg->pose.pose.position.y;
	// Truncate position values to be accurate to .01
	current_pos.x = ((long)(current_pos.x*100)) / 100.0;
	current_pos.y = ((long)(current_pos.y*100)) / 100.0;
	//ROS_INFO("Got position: %lf %lf", current_pos.x, current_pos.y);
	// Get orientation odom data
	current_orient.z = msg->pose.pose.orientation.z;
	current_orient.w = msg->pose.pose.orientation.w;
	current_heading = 2*atan2(current_orient.z, current_orient.w);
	if (current_heading < 0) {
		current_heading = TWO_PI + current_heading;
	}	
	//current_heading = floor(current_heading*100) / 100.0;
	//ROS_INFO("Got heading: %lf", current_heading);
	got_current_position = true;
}


// Array of waypoints
std::vector<geometry_msgs::Point> points;
// Current target
geometry_msgs::Point target;
// Index of target
int waypoint_index = 1;
bool done = false;

// Compares two points to see if they are equal. Since the points are doubles,
//  the point is multiplied to obtain the desired precision then cast to a long
bool comparePoints(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2){
	return ( (((long)(pt1.x*PRECISION_MULT)) == ((long)(pt2.x*PRECISION_MULT))) &&
	  		 (((long)(pt1.y*PRECISION_MULT)) == ((long)(pt2.y*PRECISION_MULT))) );
}

// Calculate the distance between two points
double dist(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
	double dx = fabs(pt1.x - pt2.x);
	double dy = fabs(pt1.y - pt2.y);
	return sqrt( (dx*dx) + (dy*dy) );
}


/**********************************************************************************/


int main(int argc, char **argv){

	// Get name of file of points from the command line
    string filename;
    if (!argv[1]) {
        ROS_ERROR("Point file not specified. Exiting.");
        return 1;
    }
    // Parse the file and store the waypoints
    ifstream pointFile(argv[1]);
    if(!pointFile) {
        ROS_ERROR("Can't open file. Exiting.");
        return 1;
    }
    string line;
    vector<geometry_msgs::Point> points;
    double tmpx, tmpy;
    while (getline(pointFile,line)) {
        istringstream is(line);
        is >> tmpx;
        is >> tmpy;
        // Truncate points if the file creater wasn't very nice
        tmpx = floor(tmpx*10)/10.0;
        tmpy = floor(tmpy*10)/10.0;
        geometry_msgs::Point temppoint;
        temppoint.x = tmpx;
        temppoint.y = tmpy;
        points.push_back(temppoint);
    }
    pointFile.close();

	// Push back the last element (the goal) twice to simplify the 
    //  path following control logic later
    points.push_back(points[points.size()-1]);
    // Get the first target (this is index 1 b/c index 0 is the start pos.)
    waypoint_index = 1;
    target = points[1];

	// Initialize the node and get the node handle  
    ros::init(argc, argv, "local_driver_node");
    ros::NodeHandle n;

    // Set up publishers
    ros::Subscriber odom_sub = n.subscribe("/Odometry", 100, odomCallback);
    //ros::Subscriber odom_sub = n.subscribe("/r1/odom", 100, odomCallback);
	// Desired trajectory vector publisher
  	ros::Publisher desired_trajectory_pub = n.advertise<geometry_msgs::Vector3>("desired_trajectory", 10);
    
	// Set up subscribers
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/local_path", 10);

    // Specify frequency to publish (Hz)
    double pub_rate = 10;
    ros::Rate loop_rate(pub_rate);
    // A count of how many messages we have sent. This is used to create a unique string for each
    int count = 0;

    // Initialize path message. this is just for visualization
    nav_msgs::Path path_msg;
	// Array of stamped poses so that we can display the path in rviz
    std::vector<geometry_msgs::PoseStamped> pose_path;
    path_msg.header.frame_id = "odom"; // This is critical for proper display in rviz!
    for(std::vector<geometry_msgs::Point>::reverse_iterator it=points.rbegin(); it!=points.rend(); ++it) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.frame_id = "odom";
        temp_pose.pose.position = *it;
        pose_path.push_back(temp_pose);
    }
	
	// The desired trajectory message
	geometry_msgs::Vector3 desired_trajectory_msg;

	// Some state variables	for navigation
	double deltaX;
	double deltaY;
	double target_heading;
	double heading_error;
	int turn_dir = 1; // 1 = Left, -1 = Right
	double turn_velocity = 0;
	double linear_velocity = 0;

	// Don't start until we've got our current position and a set of waypoints
	while ( !got_current_position) {
		ros::spinOnce();
	}

	while (ros::ok()) {
	
		// SpinOnce calls callbacks waiting to be called at this point in time
        // This will update current_pos
		ros::spinOnce();
	
		if (done) {
			goto sleep;
		}
	
		// If we're at the target point, or if we're closer to the next point 
		//   than the last, update target
		if ( (dist(current_pos, points[waypoint_index+1]) < dist(current_pos, points[waypoint_index-1])) 
				|| (dist(current_pos, points[waypoint_index]) <= ADVANCE_TARGET_RANGE) ) {
			// Check and see if we're at the goal (the last point in the vector)
			if ( comparePoints(current_pos, points[points.size()-1]) ) {
				ROS_INFO("DONE!");		
				//ROS_INFO("Position: %lf %lf", current_pos.x, current_pos.y);
				done = true;
				//vel_msg.linear.x = 0;
				//vel_msg.angular.z = 0;
				//vel_pub.publish(vel_msg);
				desired_trajectory_msg.x = 0;
                		desired_trajectory_msg.y = 0;
		                desired_trajectory_msg.z = 0;
				path_pub.publish(path_msg);
				goto sleep;
			}
			//ROS_INFO("TARGET REACHED.");
			waypoint_index++;
			target = points[waypoint_index];	
			// Remove current point from pose path
			if (!pose_path.empty()) {
				pose_path.pop_back();
			}
			ROS_INFO("New Target: %lf %lf", target.x, target.y);
		}

		// Find angle and distance to target destination
        deltaX = target.x - current_pos.x;
        deltaY = target.y - current_pos.y;
        target_heading = atan2(deltaY, deltaX);
        // Normalize +/- 2PI to 0-PI
        if (target_heading < 0) {
            target_heading = TWO_PI + target_heading;
        }
        // Calculate error
        heading_error = current_heading - target_heading;
        //ROS_INFO("Target position: %lf %lf", target.x, target.y);
        //ROS_INFO("Target heading: %lf", target_heading);
		//ROS_INFO("Angle error: %lf", heading_error);
		
		// Determine turning direction based on smallest angle. This is hard because of
		//  wrap around
		if (current_heading > target_heading) {
			heading_error = current_heading - target_heading;
			if (heading_error > M_PI) {
				turn_dir = 1; // Turn left
			} else {
				turn_dir = -1; // Turn right
			}
		} else {
			heading_error = target_heading - current_heading;
			if (heading_error < M_PI) {
				turn_dir = 1; // Turn left
			} else {
				turn_dir = -1; // Turn right
			}
		}
		// Get the magnitude of heading error
		if (heading_error > M_PI) {
			heading_error = TWO_PI - heading_error;
		}
        ROS_INFO("Angle error: %lf", turn_dir * heading_error);
	
		// Determine desired vector based on heading error
		// Positive angle is a right turn
		desired_trajectory_msg.x = 5000 * (cos(turn_dir * heading_error) * M_PI / 180);
		desired_trajectory_msg.y = 5000 * (sin(turn_dir * heading_error) * M_PI / 180);
		desired_trajectory_msg.z = 0;
		desired_trajectory_pub.publish(desired_trajectory_msg);		
	
		// Publish the path so it can be visualized
		path_msg.header.stamp = ros::Time::now();	
		path_msg.poses = pose_path;
		path_pub.publish(path_msg);

		sleep:
        // Sleeps for (1/loop_rate) seconds
        loop_rate.sleep();
        ++count;
    } // While ros::ok()

    return 0;
}


