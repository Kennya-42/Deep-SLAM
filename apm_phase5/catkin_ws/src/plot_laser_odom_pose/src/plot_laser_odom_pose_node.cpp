#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>


geometry_msgs::PoseArray pose_array;
geometry_msgs::Pose p;

void get_laser_odom_pose( const geometry_msgs::Pose2D pose_msg )
{
	pose_array.header.stamp = ros::Time::now();
	pose_array.header.frame_id = "odom";
	p.position.x = pose_msg.x;
	p.position.y = pose_msg.y;
	p.orientation = tf::createQuaternionMsgFromYaw( pose_msg.theta );
	pose_array.poses.push_back( p );	
}


int main(int argc, char **argv)
{
	ros::init( argc, argv, "plot_laser_odom_pose_node" );

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe( "pose2D", 100, get_laser_odom_pose );
	ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>( "plop", 100 );

	ros::Rate loop_rate( 100 );
	while( ros::ok() )
	{
		pub.publish( pose_array );
		ros::spinOnce();
	}
  
  return 0;
}
