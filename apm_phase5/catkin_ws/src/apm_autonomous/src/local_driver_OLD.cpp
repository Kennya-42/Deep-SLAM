/**
*	gps_driver.cpp
*
*	Alex Avery
*   aja9675@rit.edu	
*	5/1/16
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include "geodesy/utm.h"
#include "geographic_msgs/GeoPoint.h"

#define TWO_PI 2*M_PI

using namespace std;

// Create current_pos and orient as a globals so they can be set in the callback
geometry_msgs::Point current_pos; // See hw3/include/hw3/Point.h
geometry_msgs::Quaternion current_orient;
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
}


// Calculate the distance between two points
double dist(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
    double dx = fabs(pt1.x - pt2.x);
    double dy = fabs(pt1.y - pt2.y);
    return sqrt( (dx*dx) + (dy*dy) );
}


// Compares two points to see if they are equal. Since the points are doubles,
//  the point is multiplied to obtain the desired precision then cast to a long
bool comparePoints(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2){
    return ( (((long)(pt1.x*PRECISION_MULT)) == ((long)(pt2.x*PRECISION_MULT))) &&
             (((long)(pt1.y*PRECISION_MULT)) == ((long)(pt2.y*PRECISION_MULT))) );
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


	// Initialize the node and get the node handle	
	ros::init(argc, argv, "local_driver_node");
    ros::NodeHandle n;

	// Set up publishers
	ros::Subscriber odom_sub = n.subscribe("/odometry/filtered/odomlaser", 100, odomCallback);

	// Set up subscribers
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/local_path", 10);	

	// Specify frequency to publish (Hz)
    double pub_rate = 10;
    ros::Rate loop_rate(pub_rate);
    // A count of how many messages we have sent. This is used to create a unique string for each
    int count = 0;

	// Initialize path message. this is just for visualization
	nav_msgs::Path path_msg;
	std::vector<geometry_msgs::PoseStamped> pose_path;
	path_msg.header.frame_id = "/odom"; // This is critical for proper display in rviz!
	for(std::vector<geometry_msgs::Point>::reverse_iterator it=points.rbegin(); it!=points.rend(); ++it) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.frame_id = "/odom";
        temp_pose.pose.position = *it;
        pose_path.push_back(temp_pose);
    }


	while (ros::ok()) {
	
		// SpinOnce calls callbacks waiting to be called at this point in time
        // This will update current_pos
		ros::spinOnce();

		// Based on current odometry, find the closest point on the path. This could be optimized.
		float min_dist = 999999.0;
		float min_dist_index = -1;
		float current_index = 0;
		float temp_dist;
		for(std::vector<geometry_msgs::Point>::reverse_iterator it=points.rbegin(); it!=points.rend(); ++it) {
    		temp_dist = dist(current_pos, *it);
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
				min_dist_index = current_index;
			}
			current_index++;
		}
		
		// Determine the desired heading based on the closest point on the path.
					



		// Update the path and publish it
		path_msg.header.seq = count;
		path_msg.header.stamp = ros::Time::now();	
		path_msg.poses = pose_path;
        path_pub.publish(path_msg);

		// Sleeps for (1/loop_rate) seconds
        loop_rate.sleep();
        ++count;
    } // While ros::ok()

    return 0;
}


