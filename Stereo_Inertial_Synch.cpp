// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
//
// This script opens a rosbag file and saves synchronised IMU and Stereo Camera topics to a new rosbag, 
// while preserving the original frequency of the IMU compared to the cameras.
// The approximate time synchronizer filter is used from the ROS message_filters package together
// with additional buffers to achieve this.
//
// Usage: create a package in your ROS src directory called synchronize.
// $ rosrun synchronize synchronize_node
//
// You may have to remove the "protected:" above the signalMessage function in simple_filter.h
// $ sudo gedit /opt/ros/melodic/include/message_filters/simple_filter.h
//
// Remember to set the path to the rosbag you want to open and the sensor topics
// under GLOBAL VARIABLES.
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
 
using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFile = "/home/user/Documents/ROS_Workspace/rosbags/AllSensors_600x600_15fps_100Hz_1Hz_2023-02-15.bag";
std::string cam0_topic = "/video_source_0/raw";
std::string cam1_topic = "/video_source_1/raw";
std::string accel_topic = "imu/acceleration";
std::string gyro_topic = "imu/angular_velocity";

int synch_cnt, i, j, k, l = 0; 
std::vector<stereo_inertial> SynchedMsgsVec;

struct stereo_inertial {
  sensor_msgs::Image img0;
  sensor_msgs::Image img1;
  geometry_msgs::Vector3Stamped accel;
  geometry_msgs::Vector3Stamped gyro;
};



//-------------------------CALLBACKS-------------------------------------------------------
void SynchCallback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg, const geometry_msgs::Vector3Stamped::ConstPtr& accel_msg, const  geometry_msgs::Vector3Stamped::ConstPtr& gyro_msg)
// Callback for synchronizing stereo messages with imu messages - higher IMU rate gets lost 
{ 
  struct stereo_inertial StereoInertialStruct;

  // Insert synched messages into the struct
  StereoInertialStruct.img0 = *img0_msg;
  StereoInertialStruct.img0 = *img0_msg;
  StereoInertialStruct.accel = *accel_msg;
  StereoInertialStruct.gyro = *gyro_msg;

  // Insert the struct into the vector
  SynchedMsgsVec.push_back(StereoInertialStruct);

  synch_cnt += 1;
}



//-------------------------FUNCTIONS-------------------------------------------------------
void synchronizeBag(const std::string& filename, ros::NodeHandle& nh)
// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
{
  // Load unsynched rosbag
  rosbag::Bag unsynched_bag;
  unsynched_bag.open(filename, rosbag::bagmode::Read);
  std::vector<std::string> topics; // create a vector of topics to iterate through
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  topics.push_back(accel_topic);
  topics.push_back(gyro_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbagFolderPath+"StereoInertialSynched.bag", rosbag::bagmode::Write); 

  // Set up message_filters subscribers to capture messages from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(nh, accel_topic, 20); 
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(nh, gyro_topic, 20);
  
  // Create Approximate Time Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> approxTimePolicy;
  //ros::Duration maxInterval = ros::Duration(0.5,0);
  //message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>::setMaxIntervalDuration(maxInterval); // set maximum synchronization timestamp interval to (seconds, nanoseconds)
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_sub, img1_sub, accel_sub, gyro_sub);
  sync.registerCallback(boost::bind(&SynchCallback, _1, _2, _3, _4));

  // Iterate through all messages on all topics in the bag and send them to the synchronizer callback
  cout << "Writing to synched bag file. This will take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the SynchCallback
        i += 1;
    }

    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1); 
        j += 1;
    }

    if (msg.getTopic() == accel_topic)
    {
      geometry_msgs::Vector3Stamped::ConstPtr accel = msg.instantiate<geometry_msgs::Vector3Stamped>();
      if (accel != NULL)
        accel_sub.signalMessage(accel); 
        k += 1;
    }

    if (msg.getTopic() == gyro_topic)
    {
      geometry_msgs::Vector3Stamped::ConstPtr gyro = msg.instantiate<geometry_msgs::Vector3Stamped>();
      if (gyro != NULL)
        gyro_sub.signalMessage(gyro); 
        l += 1;
    }
    writeToBag(synched_bag); // write to the rosbag (disk) and empty the vector as callbacks are made to save RAM space
  }

  unsynched_bag.close();
  synched_bag.close();
  cout << "Closing both bag files." << endl;
  cout << "Total img0 callbacks = " << i << endl;
  cout << "Total img1 callbacks = " << j << endl;
  cout << "Total accel callbacks = " << k << endl;
  cout << "Total gyro callbacks = " << l << endl;
  cout << "Total synched messages = " << synch_cnt << endl;
}




void writeToBag(rosbag::Bag& synched_bag)
// Write synchronized messages to the synched rosbag
{
  sensor_msgs::Image img0_msg, img1_msg;
  geometry_msgs::Vector3Stamped accel_msg, gyro_msg;
  struct stereo_inertial StereoInertialStruct;

  if(!SynchedMsgsVec.empty())
  {
    // Get latest synched messages from the vector and then delete them from the vector 
    StereoInertialStruct = SynchedMsgsVec.front();
    SynchedMsgsVec.erase(SynchedMsgsVec.begin());

    img0_msg = StereoInertialStruct.img0;
    img1_msg = StereoInertialStruct.img1;
    accel_msg = StereoInertialStruct.accel;
    gyro_msg = StereoInertialStruct.gyro;

    // Write a synched set of messages to a rosbag
    synched_bag.write(cam0_topic, img0_msg.header.stamp, img0_msg); 
    synched_bag.write(cam1_topic, img1_msg.header.stamp, img1_msg);
    synched_bag.write(accel_topic, accel_msg.header.stamp, accel_msg); 
    synched_bag.write(gyro_topic, gyro_msg.header.stamp, gyro_msg);
  }
}




//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronize_node");
  ros::NodeHandle nh;

  synchronizeBag(rosbagFolderPath+unsynchedBagName, nh);
   
  // Register the IMU_buffer callback
  // can you register the same callback with two subscribers?
  // possibly just use /imu as the topic and then separate the topics further down the line when needed
  // but the imu and acceleration might not be exactly synched themselves?
  accel_sub.registerCallback(IMU_BufferCallback);
  gyro_sub.registerCallback(IMU_BufferCallback);

  ros::spin();
  return 0;
}
