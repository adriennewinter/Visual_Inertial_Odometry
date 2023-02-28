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
 
using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFolderPath = "/home/user/Documents/ROS_Workspace/rosbags";
std::string unsynchedBagName = "AllSensors_600x600_15fps_100Hz_1Hz_23-02-2023.bag";
std::string cam0_topic = "/video_source_0/raw";
std::string cam1_topic = "/video_source_1/raw";
std::string imu_topic = "/imu/data";

struct stereo_inertial {
  sensor_msgs::Image img0;
  sensor_msgs::Image img1;
  sensor_msgs::Imu imu;
};

int synch_cnt, i, j, k = 0; 
std::deque<stereo_inertial> SynchedMsgsBuffer, OutBuffer;
std::deque<sensor_msgs::Imu> imuBuffer;





//-------------------------CALLBACKS-------------------------------------------------------
void SynchCallback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
// Callback for synchronizing stereo messages with imu messages - higher IMU rate gets lost 
{ 
  struct stereo_inertial SynchedMsgsStruct;

  // Insert synched messages into the struct
  SynchedMsgsStruct.img0 = *img0_msg;
  SynchedMsgsStruct.img1 = *img1_msg;
  SynchedMsgsStruct.imu = *imu_msg;

  // Insert the struct into the deque
  SynchedMsgsBuffer.push_back(SynchedMsgsStruct);

  synch_cnt += 1;
}



void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// Add all IMU messages to a buffer (deque)
{
  imuBuffer.push_back(*imu_msg);
}



//-------------------------FUNCTIONS-------------------------------------------------------
void orderOutBuffer()
// Order the IMU messages and synched image-imu messages in the OutBuffer
{
  cout << "orderOutBuffer" << endl;
  struct stereo_inertial SynchedMsgsStruct, imuStruct;

  // Get latest synched message struct from the SynchedMsgsBuffer, delete it from the deque and add it to the OutBuffer
  SynchedMsgsStruct = SynchedMsgsBuffer.front();
  SynchedMsgsBuffer.pop_front();
  OutBuffer.push_back(SynchedMsgsStruct);

  // Look through the messages in the imuBuffer to find the same message that came through the synchronizer callback
  deque<sensor_msgs::Imu>::iterator imu_pos = imuBuffer.begin();
  for(int i=0; i<=imuBuffer.size(); i++){      
    if(SynchedMsgsStruct.imu.header.stamp == imuBuffer.at(i).header.stamp){        
      imuBuffer.erase(imu_pos); // this IMU message has already been added to the OutBuffer
      i--; 
      imu_pos--;
      deque<stereo_inertial>::iterator out_pos = OutBuffer.end();

      // Add any earlier IMU messages that occured before the latest synched image-imu message to the OutBuffer
      for(int j=i; j>=0; j--){               
        imuStruct.imu = imuBuffer.at(j);
        out_pos = OutBuffer.insert(out_pos, imuStruct);
        imuBuffer.erase(imu_pos);
        imu_pos--;
      }
      break;
    }
    imu_pos++;
  }
}




void writeToBag(rosbag::Bag& synched_bag)
// Write synchronized messages to the synched rosbag
{
  sensor_msgs::Image img0_msg, img1_msg;
  sensor_msgs::Imu imu_msg;
  struct stereo_inertial OutBuffStruct;

  if(!SynchedMsgsBuffer.empty())
  {
    // Order the IMU messages and synched image-imu messages in the OutBuffer
    orderOutBuffer();

    // Get the latest struct messages from OutBuffer to be written to the synched rosbag and delete the used message from OutBuffer
    cout << "writeToBag" << endl;
    for(auto OutBuffStruct=OutBuffer.begin(); OutBuffStruct<=OutBuffer.end(); OutBuffStruct++){
      img0_msg = OutBuffStruct->img0;
      img1_msg = OutBuffStruct->img1;
      imu_msg = OutBuffStruct->imu;
      OutBuffer.pop_front();

      synched_bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
      if(img0_msg.header.stamp!=ros::TIME_MIN && img1_msg.header.stamp!=ros::TIME_MIN){
        synched_bag.write(cam0_topic, img0_msg.header.stamp, img0_msg); 
        synched_bag.write(cam1_topic, img1_msg.header.stamp, img1_msg);
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
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbagFolderPath+"/"+"StereoInertialSynched.bag", rosbag::bagmode::Write); 

  // Set up message_filters subscribers to capture messages from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 20); 
  
  // Create Approximate Time Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy;
  //ros::Duration maxInterval = ros::Duration(0.5,0); // (seconds, nanoseconds)
  //message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu>::setMaxIntervalDuration(maxInterval); // set maximum synchronization timestamp difference 
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_sub, img1_sub, imu_sub);
  sync.registerCallback(boost::bind(&SynchCallback, _1, _2, _3));

  // Register the IMU Buffer Callback
  imu_sub.registerCallback(imuBufferCallback);

  // Iterate through all messages on all topics in the bag and send them to their callbacks
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
        img1_sub.signalMessage(img1); // call the SynchCallback
        j += 1;
    }

    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_sub.signalMessage(imu); // call the SynchCallback and imuBufferCallback
        k += 1;
    }

    writeToBag(synched_bag); // write to rosbag (disk) and empty the deques as callbacks are made to save RAM space
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
