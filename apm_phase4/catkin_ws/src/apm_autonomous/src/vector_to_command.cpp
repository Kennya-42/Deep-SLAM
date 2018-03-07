#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>

#define MAX_THROTTLE_PERCENT 40

// Current remote commands
float remote_cmd_throttle;
float remote_cmd_steering;
float remote_cmd_brake;

// Calculated commands from cmd_trajectory
float calculated_cmd_throttle;
float calculated_cmd_steering;
float calculated_cmd_brake;

void _callback_cmd_trajectory(const geometry_msgs::Vector3::ConstPtr& msg)
{
    calculated_cmd_throttle = sqrt(msg->x*msg->x + msg->y*msg->y);
    calculated_cmd_steering = atan2(msg->y,msg->x) * (180 / M_PI);
    calculated_cmd_brake = 0;
}

void _callback_remote_cmd_throttle(const std_msgs::Float32::ConstPtr& msg) {
    remote_cmd_throttle = msg->data;
}

void _callback_remote_cmd_steering(const std_msgs::Float32::ConstPtr& msg) {
    remote_cmd_steering = msg->data;
}

void _callback_remote_cmd_brake(const std_msgs::Float32::ConstPtr& msg) {
    remote_cmd_brake = msg->data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vector_to_command");

  ros::NodeHandle n;

  // cmd_trajectory vector
  ros::Subscriber sub_cmd_trajectory = n.subscribe("cmd_trajectory", 1, _callback_cmd_trajectory);

  // Remote input
  ros::Subscriber sub_remote_command_throttle = n.subscribe("remote_control_cmd_throttle", 1, _callback_remote_cmd_throttle);
  ros::Subscriber sub_remote_command_steering = n.subscribe("remote_control_cmd_steering", 1, _callback_remote_cmd_steering);
  ros::Subscriber sub_remote_command_brake = n.subscribe("remote_control_cmd_brake", 1, _callback_remote_cmd_brake);

  // Desired trajectory vector publisher
  ros::Publisher pub_desired_trajectory = n.advertise<geometry_msgs::Vector3>("desired_trajectory", 1); // desired_trajectory

  // Command publishers
  ros::Publisher pub_cmd_throttle = n.advertise<std_msgs::Float32>("cmd_throttle", 10);
  ros::Publisher pub_cmd_steering = n.advertise<std_msgs::Float32>("cmd_steering", 10);
  ros::Publisher pub_cmd_brake = n.advertise<std_msgs::Float32>("cmd_brake", 10);

  // Messages
  std_msgs::Float32 cmd_throttle_msg;
  std_msgs::Float32 cmd_steering_msg;
  std_msgs::Float32 cmd_brake_msg;

  geometry_msgs::Vector3 desired_trajectory_msg;

  // ROS rate
  ros::Rate loop_rate(10);

  while (ros::ok()) {

      ros::spinOnce();

      // Publish the calculated commands from our last cmd_trajectory

      // Convert calculated_cmd_throttle to MAX_THROTTLE_PERCENT scale
      cmd_throttle_msg.data = calculated_cmd_throttle;
      if (cmd_throttle_msg.data > MAX_THROTTLE_PERCENT) {
         cmd_throttle_msg.data = MAX_THROTTLE_PERCENT;
      }
      //cmd_throttle_msg.data = MAX_THROTTLE_PERCENT;
      cmd_steering_msg.data = calculated_cmd_steering;
      cmd_brake_msg.data = calculated_cmd_brake;

      pub_cmd_throttle.publish(cmd_throttle_msg);
      pub_cmd_steering.publish(cmd_steering_msg);
      pub_cmd_brake.publish(cmd_brake_msg);

      // Calculate and publish desired vector
      desired_trajectory_msg.x = remote_cmd_throttle * cos((remote_cmd_steering * M_PI) / 180);
      desired_trajectory_msg.y = remote_cmd_throttle * sin((remote_cmd_steering * M_PI) / 180);
      desired_trajectory_msg.z = 0;
      //pub_desired_trajectory.publish(desired_trajectory_msg);

      loop_rate.sleep();

  }

  return 0;
}
