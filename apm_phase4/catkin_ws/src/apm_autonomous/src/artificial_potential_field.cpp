/*
This node is created to used to extract range data from the converted pointcloud to laserscan. The range data will be used to detect obstacles and control the cart to steer away from obstacles while it is heading towards its destination.
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>

// Environment vector values
float Fx,Fy;
ros::Publisher pub_cmd_trajectory;

// Current steering angle (in deg)
float current_steering_radius;

#define MASS_OBJECT_R_SQUARED 0.1

#define MASS_OBJECT_MPWAPF 45.0
#define MPWAPF_RANGE_EXPONENT 1.1
#define SIGMA 1.2

// Gets r^2 environment force
float _get_magnitude_r_squared (float range) {
    return (MASS_OBJECT_R_SQUARED/(range * range));
}

// Returns y-effect (steering) from MPWAPF
float _get_MPWAPF(float range, float angle) {

    //float c = sqrt( (range*range) + (current_steering_radius*current_steering_radius) - (2*range*current_steering_radius) * cos(angle*M_PI/180) );

    float distance_to_path = range * sin(angle); //current_steering_radius - c;

    float force = MASS_OBJECT_MPWAPF/( pow(range, MPWAPF_RANGE_EXPONENT) * exp((distance_to_path*distance_to_path)/SIGMA) );
	//ROS_INFO("%f", force);

    if(distance_to_path > 0){
		//ROS_INFO("POSITIVE");
		return force;
    } else {
		//ROS_INFO("NEGATIVE");
       return -1 * force;		
    }
}

// Called everytime we get a laserscan, sums to create environment vector
void _callback_laserscan (const sensor_msgs::LaserScan scan) {

    int index = 0;
    float x_magnitude = 0;
    Fx = 0.0;
	Fy = 0.0;

    for(float f=scan.angle_min; f<scan.angle_max; f+=scan.angle_increment){

        x_magnitude = _get_magnitude_r_squared(scan.ranges[index]);
		Fx = Fx - x_magnitude * cos(f);
		if (scan.ranges[index] != INFINITY) {
			Fy = Fy - _get_MPWAPF(scan.ranges[index], f);
		}
		
		index++;
	}

	//ROS_INFO("%f", Fy);	

}

void _callback_desired_trajectory( const geometry_msgs::Vector3 desired_vector){

    geometry_msgs::Vector3 trajectory;

	if (desired_vector.x != 0 || desired_vector.y != 0){
		trajectory.x = desired_vector.x; // + Fx;
		trajectory.y = desired_vector.y + Fy;
	} else {
		trajectory.x = 0;
		trajectory.y = 0;
	}
	trajectory.z = 0.0;
	pub_cmd_trajectory.publish(trajectory);
}

void _callback_cart_steering( const std_msgs::Float32 steering_msg ) {
    #define STEERING_ANGLE_TO_RADIUS 108
    current_steering_radius = STEERING_ANGLE_TO_RADIUS/steering_msg.data;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "artificial_potential_field");

	ros::NodeHandle n;

	pub_cmd_trajectory = n.advertise<geometry_msgs::Vector3>("cmd_trajectory",50);

	ros::Subscriber sub_laser = n.subscribe<sensor_msgs::LaserScan>("laser", 10, _callback_laserscan);

	ros::Subscriber sub_desired_trajectory = n.subscribe<geometry_msgs::Vector3>("desired_trajectory", 10, _callback_desired_trajectory);

    ros::Subscriber sub_cart_speed = n.subscribe<std_msgs::Float32>("cart_odom_steering", 1, _callback_cart_steering);

	ros::spin();

	return 0;
}
