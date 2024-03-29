#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
ros::init(argc, argv, "vel_tf_broadcaster");
ros::NodeHandle node;

tf::TransformBroadcaster br;
tf::Transform transform;
 
ros::Rate rate(10.0);
while (node.ok()){
	transform.setOrigin( tf::Vector3(1.4859, 0, 0.965) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "vel_tf"));
	rate.sleep();
 	}
	return 0;
};
