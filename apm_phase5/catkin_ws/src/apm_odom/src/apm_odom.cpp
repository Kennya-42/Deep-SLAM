#include "ros/ros.h"

#include <std_msgs/Float32.h>
//#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265

#define LEFT_WHEEL_DIST 0.4572
#define RIGHT_WHEEL_DIST 0.4572
#define TREAD_WIDTH .95

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
long _CurrentRightEncoderTicks = 0;
ros::Time current_time_encoder, last_time_encoder;
//double DistancePerCount = (PI * 0.4572) / 100 ;
//double TreadWidth = 1.0;
//double left_wheel_scale =0.9999;

double x;
double y;
double th;

double vx;
//double vy;
double vth;
double deltaLeftTicks;
double deltaRightTicks;
double deltaLeft;
double deltaRight;

void LeftWheelCallback(const std_msgs::Float32 ticks) {
  
  current_time_encoder = ros::Time::now();

  deltaLeftTicks = ticks.data - _PreviousLeftEncoderCounts;
  //deltaRightTicks = _CurrentRightEncoderTicks - _PreviousRightEncoderCounts;

  deltaLeft = deltaLeftTicks * (PI * LEFT_WHEEL_DIST) / 100.0; // (current_time_encoder - last_time_encoder).toSec();
  //deltaRight = deltaRightTicks * (PI * RIGHT_WHEEL_DIST) / 100.0; // (current_time_encoder - last_time_encoder).toSec();

  //ROS_INFO("  L ticks: %f", ticks.data);
  //ROS_INFO("  R ticks: %f", _CurrentRightEncoderTicks);
  //ROS_INFO("  L deltaLeftTicks: %f", deltaLeftTicks);
  //ROS_INFO("  R deltaRightTicks: %f", deltaRightTicks);
  //ROS_INFO("  L dist: %f", DistancePerCount);
  //ROS_INFO("  R dist: %f", deltaRight);


  _PreviousLeftEncoderCounts = ticks.data;
  //_PreviousRightEncoderCounts = _CurrentRightEncoderTicks;
  last_time_encoder = current_time_encoder;
}

void RightWheelCallback(const std_msgs::Float32 ticks) {

  deltaRightTicks = ticks.data - _PreviousRightEncoderCounts;

  deltaRight = deltaRightTicks * (PI * RIGHT_WHEEL_DIST) / 100.0; // (current_time_encoder - last_time_encoder).toSec();

  _PreviousRightEncoderCounts = ticks.data;
  
  _CurrentRightEncoderTicks = ticks.data;
  
}

double NormalizeAngle(double angle) {
  
  while(angle > PI)
    angle -= 2.0 * PI;

  while(angle < -PI)
    angle += 2.0 * PI;

  return angle;

}



int main(int argc, char **argv) {
  
  ros::init(argc, argv, "apm_odom");
  ros::NodeHandle n;
  ros::Subscriber subLeftWheel = n.subscribe("sensor_wheel_left", 100, LeftWheelCallback);
  ros::Subscriber subRightWheel = n.subscribe("sensor_wheel_right", 100, RightWheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel/odom", 50);   
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(20);
  while(n.ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    double dist = (deltaLeft + deltaRight) / 2.0;
    double dth = 0.0;
    //TODO find better way to determine going straight, this means slight deviation is accounted
    // I stole this from the roboclaw driver so the todo is still valid
    if (deltaLeft == deltaRight) {
      dth = 0.0;
      x += dist * cos(th);
      y += dist * sin(th);
    }
    else {
        dth = (deltaRight - deltaLeft) / (TREAD_WIDTH);
        double r = dist / dth;
        x += r * (sin(dth + th) - sin(th));
        y -= r * (cos(dth + th) - cos(th));

        th = NormalizeAngle(th + dth);
    }

    if (abs(dt) < 0.000001) {
      vx = 0.0;
      vth = 0.0;
    }
        
    else {
        vx = dist / dt;
        vth = dth / dt;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;  // this cannot ever be non-zero in this vehicle
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};


    //publish the message
    odom_pub.publish(odom);

    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /*current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    //ROS_INFO("Theta: %f", th);
    //ROS_INFO("  X: %f", x);
    //ROS_INFO("  Y: %f", y);
    //ROS_INFO("  dX: %f", delta_x);
    //ROS_INFO("  dY: %f", delta_y);


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);*/

    last_time = current_time;
  	ros::spinOnce();
    r.sleep();
  }
}
