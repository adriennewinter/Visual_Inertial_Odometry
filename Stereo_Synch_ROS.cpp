// This script opens a rosbag file and saves synchronised camera topics to a struct
// Adapted from http://wiki.ros.org/rosbag/Cookbook#Analyzing_Stereo_Camera_Data

#include <boost/foreach.hpp>
#include <deque>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h>

//-------------------------DEFINE GLOBAL VARIABLES-----------------------------------------------------
// Create double ended queues to hold the synchronized images 
std::deque<sensor_msgs::Image> imgQueue0;
std::deque<sensor_msgs::Image> imgQueue1;
// Define the camera ROS topics
std::string cam0_topic = "video_source_0/raw";
std::string cam1_topic = "video_source_1/raw";



//-------------------------FUNCTIONS-------------------------------------------------------
// Callback for synchronizing stereo messages and saving them in a struct
void stereoSynchCallback(const sensor_msgs::Image::ConstPtr& img0, const sensor_msgs::Image::ConstPtr& img1) 
{
  imgQueue0.push_back(*img0); // may need to access the data in the ptr
  imgQueue1.push_back(*img1);

  // maybe just write to the new bag here?

}
 
 

// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
void readBag(const std::string &filename, ros::NodeHandle& nh)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  
  // create a vector of topics to iterate through
  std::vector<std::string> topics;
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Set up message_filters subscribers to capture images from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  
  // Use time synchronizer to synchronize images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img0_sub, img1_sub, 25);
  sync.registerCallback(boost::bind(&stereoSynchCallback, _1, _2));
  
  // Iterate through all messages on all topics in the bag and send them to the synchronizer
  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (msg.getTopic() == cam0_topic || ("/" + msg.getTopic() == cam0_topic))
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the stereoSynchCallback - had to remove the "protected:" with sudo gedit /opt/ros/melodic/include/message_filters/simple_filter.h
    }
    
    if (msg.getTopic() == cam1_topic || ("/" + msg.getTopic() == cam1_topic))
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1);
    }
  }
  bag.close();
}



void writeSynchedBag()
{
  // Use the image queues to write to a new rosbag if possibile
}



//-------------------------MAIN-------------------------------------------------------
//TODO add argc and argv implementation for rosbag path/name
int main(int argc, char** argv)
{
   // Create a ROS node
   ros::init(argc, argv, "synchronizer_node");
   ros::NodeHandle nh;
 
   readBag("~/Documents/ROS_Workspace/rosbags/AllSensors_600x600_15fps_100Hz_1Hz_2023-02-15.bag", nh);
   //writeSynchedBag();
 
   ros::spin(); // is this needed?
   return 0;
}

