// This script opens a rosbag file and saves synchronised camera topics to a struct
// Adapted from http://wiki.ros.org/rosbag/Cookbook#Analyzing_Stereo_Camera_Data

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
// Create double ended queues to hold the synchronized images 
std::deque<sensor_msgs::Image> imgQueue0;
std::deque<sensor_msgs::Image> imgQueue1;
// Number of synched pairs counter
int synchs = 0;


//-------------------------FUNCTIONS-------------------------------------------------------
// Callback for synchronizing stereo messages and saving them in a queue
void stereoSynchCallback(const sensor_msgs::Image::ConstPtr& img0, const sensor_msgs::Image::ConstPtr& img1) 
{
  synchs += 1;
  
  //imgQueue0.push_back(*img0); 
  //imgQueue1.push_back(*img1);
  
  //cout << "imgQueue0 size = " << imgQueue0.size() << endl;
  //cout << "imgQueue1 size = " << imgQueue1.size() << endl;
  // maybe just write to the new bag here?

}
 
 

// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
void readBag(const std::string& filename, ros::NodeHandle& nh)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  cout << "Opening bag file." << endl;
  
  // create a vector of topics to iterate through
  std::string cam0_topic = "/video_source_0/raw";
  std::string cam1_topic = "/video_source_1/raw";
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

  // Iterate through all messages on all topics in the bag and send them to the synchronizer
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    cout << "entered foreach loop" << endl;
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the stereoSynchCallback - had to remove the "protected:" above signalMessage function in $sudo gedit /opt/ros/melodic/include/message_filters/simple_filter.h
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
  cout << "Total img0_sub callbacks = " << i << endl;
  cout << "Total img1_sub callbacks = " << j << endl;
  bag.close();
  cout << "Closing the bag file." << endl;
  cout << "Entered callback " << synchs << " times." << endl;
  //cout << "Final imgQueue0 size = " << imgQueue0.size() << endl;
  //cout << "Final imgQueue1 size = " << imgQueue1.size() << endl;
}



//-------------------------MAIN-------------------------------------------------------
//TODO add argc and argv implementation for rosbag path/name
int main(int argc, char** argv)
{
   // Create a ROS node
   ros::init(argc, argv, "synchronizer_node");
   ros::NodeHandle nh;
   cout << "Created ROS synchronizer_node." << endl;
 
   std::string rosbagFile = "/home/user/Documents/ROS_Workspace/rosbags/AllSensors_600x600_15fps_100Hz_1Hz_2023-02-15.bag";
   readBag(rosbagFile, nh);
 
   ros::spin(); 
   return 0;
}

