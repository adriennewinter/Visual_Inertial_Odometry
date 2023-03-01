// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
//
// This script opens a rosbag file and saves synchronised IMU, Stereo Camera and Pressure Sensor
// topics to a new rosbag, while preserving the original frequency of each sensor.
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
#include <cstddef>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages
#include <std_msgs/String.h> // pressure sensor messages
 
using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFolderPath = "/home/user/Documents/ROS_Workspace/rosbags";
std::string unsynchedBagName = "AllSensors_600x600_15fps_100Hz_1Hz_23-02-2023.bag";
std::string cam0_topic = "/video_source_0/raw";
std::string cam1_topic = "/video_source_1/raw";
std::string imu_topic = "/imu/data";
std::string prs_topic = "/ezo_prs_pressure";

struct stereo_inertial {
  sensor_msgs::Image img0;
  sensor_msgs::Image img1;
  sensor_msgs::Imu imu;
  std_msgs::String prs;
};

int synch_cnt, i, j, k, l = 0; 
std::deque<stereo_inertial> SynchedMsgsBuffer;
std::deque<sensor_msgs::Imu> imuBuffer;
std::deque<std_msgs::String> prsBuffer;




//-------------------------CALLBACKS-------------------------------------------------------
void StereoInertialSynch(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg)
// Callback for synchronizing stereo messages with imu messages - higher IMU rate gets lost 
{ 
  struct stereo_inertial SynchedMsgsStruct;

  // Insert synched messages into the struct
  SynchedMsgsStruct.img0 = *img0_synch_msg;
  SynchedMsgsStruct.img1 = *img1_synch_msg;
  SynchedMsgsStruct.imu = *imu_synch_msg;

  // Insert the struct into the deque
  SynchedMsgsBuffer.push_back(SynchedMsgsStruct);

  synch_cnt += 1;
}



void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// Add all IMU messages to a buffer (deque)
{
  imuBuffer.push_back(*imu_msg);
}



void pressureBufferCallback(const std_msgs::String::ConstPtr& prs_msg)
// Add all pressure sensor messages to a buffer (deque)
{
    prsBuffer.push_back(*prs_msg);
}



//-------------------------FUNCTIONS-------------------------------------------------------
void writeToBag(rosbag::Bag& synched_bag)
// Write synchronized messages to the synched rosbag
{
  sensor_msgs::Image img0_synch_msg, img1_synch_msg;
  sensor_msgs::Imu imu_synch_msg;
  struct stereo_inertial SynchedMsgsStruct;

  if(!SynchedMsgsBuffer.empty())
  {
    // Get latest synched message struct from the SynchedMsgsBuffer and delete it from the deque
    SynchedMsgsStruct = SynchedMsgsBuffer.front();
    img0_synch_msg = SynchedMsgsStruct.img0;
    img1_synch_msg = SynchedMsgsStruct.img1;
    imu_synch_msg = SynchedMsgsStruct.imu;
    SynchedMsgsBuffer.pop_front();
    
    // Look through the messages in the imuBuffer to find the same message that came through the synchronizer callback 
    for(int i=0; i<=imuBuffer.size(); i++)
    {     
      if(imu_synch_msg.header.stamp == imuBuffer.at(i).header.stamp)
      {
        // Write any earlier IMU messages that occured before the latest synched image-imu message to the rosbag
        while(imuBuffer.front().header.stamp != imu_synch_msg.header.stamp)
        { 
          synched_bag.write(imu_topic, imuBuffer.front().header.stamp, imuBuffer.front());
          imuBuffer.pop_front();
        }
        imuBuffer.pop_front(); // remove the synched IMU message from the buffer so we don't add it twice
        
        // Write the image-imu synched message to the rosbag
        synched_bag.write(imu_topic, imu_synch_msg.header.stamp, imu_synch_msg);
        synched_bag.write(cam0_topic, img0_synch_msg.header.stamp, img0_synch_msg); 
        synched_bag.write(cam1_topic, img1_synch_msg.header.stamp, img1_synch_msg);
        break;
      }
    }
  }
}




void synchronizeBag(const std::string& filename, ros::NodeHandle& nh)
// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
{
  // Load unsynched rosbag
  rosbag::Bag unsynched_bag;
  unsynched_bag.open(filename, rosbag::bagmode::Read);
  std::vector<std::string> topics; // create a vector of topics to iterate through
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  topics.push_back(imu_topic);
  topics.push_back(prs_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbagFolderPath+"/"+"StereoInertialSynched.bag", rosbag::bagmode::Write); 

  // Set up message_filters subscribers to capture messages from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 20); 
  message_filters::Subscriber<std_msgs::String> prs_sub(nh, prs_topic, 5); 
  
  // Create Approximate Time Synchronizer for the IMU and Camera message synching
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy;
  //ros::Duration maxInterval = ros::Duration(0.5,0); // (seconds, nanoseconds)
  //message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu>::setMaxIntervalDuration(maxInterval); // set maximum synchronization timestamp difference 
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_sub, img1_sub, imu_sub);
  sync.registerCallback(boost::bind(&StereoInertialSynch, _1, _2, _3));

  // Register the IMU and pressure sensor buffer callbacks
  imu_sub.registerCallback(imuBufferCallback);
  prs_sub.registerCallback(pressureBufferCallback);

  // Iterate through all messages on all topics in the bag and send them to their callbacks
  cout << "Writing to synched bag file. This will take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the StereoInertialSynch
        i += 1;
    }

    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1); // call the StereoInertialSynch
        j += 1;
    }

    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_sub.signalMessage(imu); // call the StereoInertialSynch and imuBufferCallback
        k += 1;
    }

    if (msg.getTopic() == prs_topic)
    {
      std_msgs::String::ConstPtr prs = msg.instantiate<std_msgs::String>();
      if (prs != NULL)
        prs_sub.signalMessage(prs); // call the pressureBufferCallback
        l += 1;
    }

    writeToBag(synched_bag); // write to rosbag (disk) and empty the deques as callbacks are made to save RAM space
  }

  // Write any remaining IMU messages to bag after all synched messages are written
  if(SynchedMsgsBuffer.empty() && synch_cnt!=0)
  {
    while(!imuBuffer.empty())
    {
      synched_bag.write(imu_topic, imuBuffer.front().header.stamp, imuBuffer.front());
      imuBuffer.pop_front();
    }
  }

  unsynched_bag.close();
  synched_bag.close();
  cout << "Closing both bag files." << endl;
}



//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronize_node");
  ros::NodeHandle nh;

  synchronizeBag(rosbagFolderPath+"/"+unsynchedBagName, nh);

  cout << "Total img0 callbacks = " << i << endl;
  cout << "Total img1 callbacks = " << j << endl;
  cout << "Total imu callbacks = " << k << endl;
  cout << "Total synched messages = " << synch_cnt << endl;
  cout << "Press Ctrl+C to kill the node." << endl;

  ros::spin();
  return 0;
}