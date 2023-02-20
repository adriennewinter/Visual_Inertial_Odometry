// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
//
// This script opens a rosbag file and saves synchronised camera image topics to a new rosbag.
// Usage: create a package in your ROS src directory called stereoSynch.
// $ rosrun StereoSynch StereoSynch_node
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
#include <sensor_msgs/Image.h>

using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFile = "/home/user/Documents/ROS_Workspace/rosbags/AllSensors_600x600_15fps_100Hz_1Hz_2023-02-15.bag";
std::string cam0_topic = "/video_source_0/raw";
std::string cam1_topic = "/video_source_1/raw";
int synchs_cnt = 0; // Synched image pairs counter


//-------------------------FUNCTIONS-------------------------------------------------------
// Callback for synchronizing stereo messages and saving them in a queue
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
 
 

// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
void readBag(const std::string& filename, ros::NodeHandle& nh)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  cout << "Opening bag file." << endl;
  
  // create a vector of topics to iterate through
  std::vector<std::string> topics;
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  
  rosbag::View rosbagView(bag, rosbag::TopicQuery(topics));

  // Set up message_filters subscribers to capture images from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  
  // Use an approximate time synchronizer to synchronize image messages
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approxTimePolicy;
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_sub, img1_sub);
  sync.registerCallback(boost::bind(&stereoSynchCallback, _1, _2));
  
  int i = 0;
  int j = 0;

  // Iterate through all messages on all topics in the bag and send them to the synchronizer callback
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the stereoSynchCallback 
        i += 1;
    }
    
    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1);
        j += 1;
    }
  }
  bag.close();
  cout << "Closing the bag file." << endl;
}



//-------------------------MAIN-------------------------------------------------------
//TODO add commandline implementation for rosbag path/name and image topic names
int main(int argc, char** argv)
{
   // Create a ROS node
   ros::init(argc, argv, "synchronizer_node");
   ros::NodeHandle nh;

   readBag(rosbagFile, nh);

   cout << "Total img0 callbacks = " << i << endl;
   cout << "Total img1 callbacks = " << j << endl;
   cout << "Total synched messages " << synchs_cnt << endl;
 
   ros::spin(); 
   return 0;
}

