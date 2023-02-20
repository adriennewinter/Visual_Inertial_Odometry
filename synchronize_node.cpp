// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
//
// This script opens a rosbag file and saves synchronised sensor topics to a new rosbag, while
// preserving original frequencies of sensors with different Hz.
// The approximate time synchronizer filter is used from the ROS message_filters package together
// with additional buffers.
//
// Usage: create a package in your ROS src directory called synchronize.
// $ rosrun synchronize synchronize_node
//
// You may have to remove the "protected:" above the signalMessage function in simple_filter.h
// $ sudo gedit /opt/ros/melodic/include/message_filters/simple_filter.h
//
// Remember to set the path to the rosbag you want to open and the two camera image/video topic
// variables under GLOBAL VARIABLES.
// -----------------------------------------------------------------------------------------

#include <boost/foreach.hpp>
#include <deque>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <geometry_msgs/Vector3Stamped.h> // accelerometer and gyro messages
#include <std_msgs/String.h> // pressure sensor messages
 
using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFile = "/home/user/Documents/ROS_Workspace/rosbags/AllSensors_600x600_15fps_100Hz_1Hz_2023-02-15.bag";
std::string cam0_topic = "/video_source_0/raw";
std::string cam1_topic = "/video_source_1/raw";
int synchs_cnt = 0; // Synched image pairs counter


//-------------------------FUNCTIONS-------------------------------------------------------
// Callback for synchronizing stereo messages
void stereoSynchCallback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg) 
{
  synchs_cnt += 1;
  
  // Write synchronised image messages to a new rosbag
  rosbag::Bag bag;
  bag.open("synchedStereoBag.bag", rosbag::bagmode::Write);
  
  // does the synchronizer filter already set the same timestamp? 
  // Does it over-write the originals with a new stamp?
  // should I set both messages with the same timestamp and choose one of the messages? img0->header.stamp;
  bag.write(cam0_topic, img0->header.stamp, *img0_msg); 
  bag.write(cam1_topic, img1->header.stamp, *img1_msg);
}
 
 


int main(int argc, char** argv)
{
   ros::init(argc, argv, "synchronizer_node");
   ros::NodeHandle nh;
 
   // Create message_filter Subscribers
   //last number is a message buffer/queue which can help prevent messages from being thrown away if coming in faster than can be processed
   message_filters::Subscriber<sensor_msgs::Image> image0_sub(nh, "video_source_0/raw", 10); 
   message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, "video_source_1/raw", 10);
   message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(nh, "imu/acceleration", 20); 
   message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(nh, "imu/angular_velocity", 20);
   message_filters::Subscriber<std_msgs::String> pressure_sub(nh, "ezo_prs_pressure", 1);
   
   // Register the IMU_buffer callback
   // can you register the same callback with two subscribers?
   accel_sub.registerCallback(IMU_BufferCallback);
   gyro_sub.registerCallback(IMU_BufferCallback);
   
   // Create the synchronizer filter
   typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, std_msgs::String> ApproxTimeSyncPolicy;
   Synchronizer<ApproxTimeSyncPolicy> sync(ApproxTimeSyncPolicy(100), image0_sub, image1_sub, accel_sub, gyro_sub, pressure_sub);
   sync.registerCallback(boost::bind(&SynchronizerCallback, _1, _2, _3, _4, _5));

   //Create ROS publishers here for publishing the synchronised topics to then rosbag record as synched?
 
   ros::spin();
   return 0;
}
