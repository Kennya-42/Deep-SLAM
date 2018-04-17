#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"


sensor_msgs::PointCloud new_pc;


void _pc2_to_pc_callback( const sensor_msgs::PointCloud2& msg )
{	
	sensor_msgs::convertPointCloud2ToPointCloud( msg, new_pc );
}


int main(int argc, char **argv)
 {
 	ros::init( argc, argv, "apm_pc2_pc");
	ros::NodeHandle n;

// 	// Setup subscriber
	ros::Subscriber sub = n.subscribe( "velodyne_points", 10, _pc2_to_pc_callback );

// 	// Setup publisher
	ros::Publisher pc2_to_pc_pub = n.advertise<sensor_msgs::PointCloud>( "velodyne_points_pc", 10 );

	ros::Rate loop_rate(10);
	int count = 0;

	while( ros::ok() )
	{
		ros::spinOnce();
		pc2_to_pc_pub.publish( new_pc );
		loop_rate.sleep();
	}

  return 0;
}
