/*
 *
 *  Created on: Feb 5, 2018
 *      Author: Robert Relyea rer3378@rit.edu
 *
 */

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Safezone includes
#include "classifier.hpp"
#include "ipm.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

typedef enum pc_params
{
  pcwidth = 80,
  pcheight = 45,
}pc_params;

// Safe area mode.
// 0 = road safe
// 1 = sidewalk safe
// 2 = road and sidewalk safe
float mode = 0.0;

void modeCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mode = msg->data;
}

int main(int argc, char** argv) {

  // ROS node initialization
  ros::init(argc, argv, "safezone");
  ros::NodeHandle n;
  ros::Publisher safe_pub = n.advertise<PointCloudT>("safezone_pc", 1);
  ros::Publisher unsafe_pub = n.advertise<PointCloudT>("unsafezone_pc", 1);
  ros::Subscriber mode_sub = n.subscribe("apm_safemode", 1, modeCallback);

  // ENet initialization
  string model_file   = "/home/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_model.prototxt";
  string trained_file = "/home/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_weights.caffemodel"; //for visualization

  Classifier classifier(model_file, trained_file);
  ipm mapper;

  string LUT_file = "/home/apm/apm_phase5/safezone.png";

  // Video capture initialization
  // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-02-171452.webm");
  cv::VideoCapture cap("/dev/video0");
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
  // cv::VideoCapture cap("/home/apm/Videos/1.mp4");

  cv::Mat img; // Create mat to hold new frames
  cv::Mat enet_frame, output_frame;

  while (ros::ok())
  {
    std::cout << "Grabbing new frame" << std::endl;
    cap >> img;  // Grab new frame
    cv::resize(img, img, cv::Size(frameWidth, frameHeight));


    enet_frame = classifier.Predict(img, LUT_file);
    
    cv::resize(enet_frame, enet_frame, cv::Size(frameWidth, frameHeight));
    

    // Remove lens distortion
    cv::resize(enet_frame, output_frame, cv::Size(1920, 1080));
    output_frame = mapper.removeDistortion(output_frame);
    // Apply IPM
    output_frame = mapper.mapPerspective(output_frame);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unsafe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    safe_cloud->header.frame_id = "/camera";
    safe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    unsafe_cloud->header.frame_id = "/camera";
    unsafe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    // Construct pointcloud from perspective mapped image

    cv::Mat cloud_frame;
    cv::resize(output_frame, cloud_frame, cv::Size(pcwidth, pcheight));

    for (int r = 0; r < pcheight; ++r)
    {
      for (int c = 0; c < pcwidth; ++c)
      {

        if (mode == 0) // Roads safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[0] != 0 || cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else if (cloud_frame.at<cv::Vec3b>(r,c)[1] == 255) // Green mask, road
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

        else if (mode == 1) // Sidewalks safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[1] != 0 || cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else if (cloud_frame.at<cv::Vec3b>(r,c)[0] == 255) // Blue mask, sidewalk
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

        else // Both roads and sidewalks safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else // Blue mask, sidewalk
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

      }
    }
    safe_cloud->width = (int) safe_cloud->points.size();
    safe_cloud->height = 1;

    unsafe_cloud->width = (int) unsafe_cloud->points.size();
    unsafe_cloud->height = 1;

    safe_pub.publish(safe_cloud);
    unsafe_pub.publish(unsafe_cloud);

    cv::Mat top_preview, bottom_preview, preview;
    cv::Mat overlay;

    // Overlay segmentation mask onto original image
    cv::addWeighted(img, 1, enet_frame, 0.7, 0, overlay);

    cv::hconcat(img, enet_frame, top_preview);
    cv::hconcat(overlay, output_frame, bottom_preview);
    cv::vconcat(top_preview, bottom_preview, preview);

    // cv::imshow("top_preview", top_preview);
    cv::resize(preview, preview, cv::Size(), 0.80, 0.80);
    cv::imshow("preview", preview);
    // cv::imshow("Original Frame", img);
    // cv::imshow("ENet Output Frame", enet_frame);
    // cv::imshow("Perspective Mapped Frame", output_frame);

    cv::waitKey(1);
    ros::spinOnce();
  }
}
