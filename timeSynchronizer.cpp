// This ROS node uses the Time Synchronizer filter from the message_filters ROS package. We use the Subscriber filter from the same package as a wrapper to let the Time Synchroniser filter access the desired ROS topics.

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <geometry_msgs/Vector3Stamped.h> // accelerometer and gyro messages
#include <std_msgs/String.h> // pressure sensor messages
 
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
 

void SynchronizerCallback(const ImageConstPtr& image0_msg, const ImageConstPtr& image1_msg, const Vector3StampedConstPtr& imu_accel_msg, const Vector3StampedConstPtr& imu_gyro_msg, const StringConstPtr& prs_msg) 
{
   // I think that all topics published here will be synchronised - or do I not even need to re-publish them???
   // Topics: IMU_sync, Cam0_sync, Cam1_sync, Prs_sync
   // Does the IMU keep its higher rate?
   ROS_INFO("Pressure is: [%s]", msg->data.c_str());
   ROS_INFO("Image0 is: [%s]", image0->data.c_str());//this might not work for images - find a way to print timestamps?
}



void IMU_BufferCallback(const Vector3StampedConstPtr& imu_accel_msg, const Vector3StampedConstPtr& imu_gyro_msg)
{

}



void mergeData()
{

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
