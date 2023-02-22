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
std::string accel_topic = "imu/acceleration";
std::string gyro_topic = "imu/angular_velocity";
std::string prs_topic = "ezo_prs_pressure";

std::queue<sensor_msgs::Image> img0_synched_queue, img1_synched_queue, stereo_synched_queue;
int synchs_cnt, i, j = 0; 



//-------------------------CALLBACKS-------------------------------------------------------
void stereoSynchCallback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg)
// Callback for synchronizing stereo messages
{ 
  sensor_msgs::Image temp_arr = [*img0_msg, *img1_msg];
  stereo_synched_queue.push(temp_arr);

  //img0_synched_queue.push(*img0_msg);
  //img1_synched_queue.push(*img1_msg);

  synchs_cnt += 1;
}



void Synch1Callback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg, const std_msgs::String::ConstPtr& prs_msg)
// Callback for synchronizing stereo messages with pressure sensor 
{ 
  

  synchs_cnt += 1;
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
  topics.push_back(prs_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbagFolderPath+"MultiSensorSynched.bag", rosbag::bagmode::Write); 

  // Set up message_filters subscribers to capture messages from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(nh, accel_topic, 20); 
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(nh, gyro_topic, 20);
  message_filters::Subscriber<std_msgs::String> prs_sub(nh, prs_topic, 1);
  
  //------------Create Approximate Time Synchronizers------------
  // left image - right image 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approxTimePolicy;
  //ros::Duration maxInterval = ros::Duration(0.5,0);
  //message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>::setMaxIntervalDuration(maxInterval); // set maximum synchronization timestamp interval to (seconds, nanoseconds)
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_sub, img1_sub);
  sync.registerCallback(boost::bind(&stereoSynchCallback, _1, _2));

  // stereo - pressure sensor
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, std_msgs::String> approxTimePolicy1;
  message_filters::Synchronizer<approxTimePolicy1> sync1(approxTimePolicy1(100), img0_sub, img1_sub, prs_sub);
  sync1.registerCallback(boost::bind(&Synch1Callback, _1, _2, _3));

  // stereo - pressure sensor - imu
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, std_msgs::String> approxTimePolicy2;
  message_filters::Synchronizer<approxTimePolicy2> sync2(approxTimePolicy2(100), img0_sub, img1_sub, prs_sub);
  sync2.registerCallback(boost::bind(&Synch2Callback, _1, _2, _3));
  //--------------------------------------------------------------

  // Iterate through all messages on all topics in the bag and send them to the synchronizer callback
  cout << "Writing to synched bag file. This will take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call all registered callbacks on this subscriber
        i += 1;
    }

    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1); 
        j += 1;
    }
    writeToBag(synched_bag); // write to the rosbag (disk) and empty the image queues as callbacks are made to save RAM space
  }
  unsynched_bag.close();
  synched_bag.close();
  cout << "Closing both bag files." << endl;
  cout << "Total img0 callbacks = " << i << endl;
  cout << "Total img1 callbacks = " << j << endl;
  cout << "Total synched messages = " << synchs_cnt << endl;
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
