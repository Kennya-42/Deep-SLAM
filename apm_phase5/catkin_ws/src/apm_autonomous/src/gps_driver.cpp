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
	// Array of UTM waypoints
    vector<geodesy::UTMPoint> utm_points;
    double tmplon, tmplat;
	char dummy_comma;
    while (getline(pointFile,line)) {
        istringstream is(line);
        is >> tmplon;
		is >> dummy_comma;
        is >> tmplat;
		// Read in Geopoint
		geographic_msgs::GeoPoint geopoint;
        geopoint.longitude = tmplon;
        geopoint.latitude = tmplat;
		geopoint.altitude = 0.0;
		geodesy::UTMPoint temp_utmpoint;
		// Convert to UTM
        geodesy::convert(geopoint, temp_utmpoint);
		//ROS_INFO("Read Geopoint (lat,lon): %lf, %lf", tmplat, tmplon);
		//ROS_INFO("Converted to UTM (northing,easting): %lf, %lf", 
		//			temp_utmpoint.northing, temp_utmpoint.easting);
        utm_points.push_back(temp_utmpoint);
    }
    pointFile.close();

	// Initialize the node and get the node handle	
	ros::init(argc, argv, "gps_driver_node");
    ros::NodeHandle n;
	// Set up publishers
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/utm_path", 10);	
	
	// Set up subscription
	//ros::Subscriber odom_sub = n.subscribe("/r1/odom", 100, odomCallback);
	
	// Specify frequency to publish (Hz)
    double pub_rate = 10;
    ros::Rate loop_rate(pub_rate);
    // A count of how many messages we have sent. This is used to create a unique string for each
    int count = 0;

	std::vector<geometry_msgs::PoseStamped> pose_path;
	// Initialize messages to publish
	nav_msgs::Path path_msg;
	path_msg.header.frame_id = "/utm"; // This is critical for proper display in rviz!
	for(std::vector<geodesy::UTMPoint>::reverse_iterator it=utm_points.rbegin(); it!=utm_points.rend(); ++it) {
        geometry_msgs::PoseStamped temp_pose;
        //temp_pose.frame_id = "map";
        temp_pose.pose.position.x = it->easting;
        temp_pose.pose.position.y = it->northing;
        pose_path.push_back(temp_pose);
    }


	while (ros::ok()) {
	
		// SpinOnce calls callbacks waiting to be called at this point in time
        // This will update current_pos
		ros::spinOnce();

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


