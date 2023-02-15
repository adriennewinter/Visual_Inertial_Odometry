// This script opens a rosbag file and saves synchronised camera topics to a struct

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// A struct to hold the synchronized camera stereo data 
class StereoData
{
public:
  sensor_msgs::Image::ConstPtr image0, image1;
  sensor_msgs::CameraInfo::ConstPtr cam_info_0, cam_info_1;
  
  StereoData(const sensor_msgs::Image::ConstPtr &img0, 
             const sensor_msgs::Image::ConstPtr &img1, 
             const sensor_msgs::CameraInfo::ConstPtr &info0, 
             const sensor_msgs::CameraInfo::ConstPtr &info1) :
    image0(img0),
    image1(img1),
    cam_info_0(info0),
    cam_info_1(info1)
  {}
};


// Inherits from message_filters::SimpleFilter<M> to use protected newMessage function when reading from the rosbag
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};


// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr &img0, 
              const sensor_msgs::Image::ConstPtr &img1, 
              const sensor_msgs::CameraInfo::ConstPtr &info0,
              const sensor_msgs::CameraInfo::ConstPtr &info1)
{
  StereoData sd(img0, img1, info0, info1); // struct - should this not be stereo_dataset_?

  // stereo_dataset_ is class variable to store data
  stereo_dataset_.push_back(sd);
}
 

// Load rosbag and Synchronize the image topics before saving the messages into a struct
void Main(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  
  std::string cam0 = image_ns_ + "/0";
  std::string cam1 = image_ns_ + "/1";
  std::string cam0_image = cam0 + "/image1aw";
  std::string cam1_image = cam1 + "/image1aw";
  std::string cam0_info = cam0 + "/camera_info";
  std::string cam1_info = cam1 + "/camera_info";
  
  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(cam0_image);
  topics.push_back(cam1_image);
  topics.push_back(cam0_info);
  topics.push_back(cam1_info);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> img0_sub, img1_sub;
  BagSubscriber<sensor_msgs::CameraInfo> info0_sub, info1_sub;
  
  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync(img0_sub, img1_sub, info0_sub, info1_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == cam0_image || ("/" + m.getTopic() == cam0_image))
    {
      sensor_msgs::Image::ConstPtr img0 = m.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.newMessage(img0);
    }
    
    if (m.getTopic() == cam1_image || ("/" + m.getTopic() == cam1_image))
    {
      sensor_msgs::Image::ConstPtr img1 = m.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.newMessage(img1);
    }
    
    if (m.getTopic() == cam0_info || ("/" + m.getTopic() == cam0_info))
    {
      sensor_msgs::CameraInfo::ConstPtr info0 = m.instantiate<sensor_msgs::CameraInfo>();
      if (info0 != NULL)
        info0_sub.newMessage(info0);
    }
    
    if (m.getTopic() == cam1_info || ("/" + m.getTopic() == cam1_info))
    {
      sensor_msgs::CameraInfo::ConstPtr info1 = m.instantiate<sensor_msgs::CameraInfo>();
      if (info1 != NULL)
        info1_sub.newMessage(info1);
    }
  }
  bag.close();
}