#ifndef STEREO_PROCESSOR_H_
#define STEREO_PROCESSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

namespace viso_isl_ros
{

class StereoProcessor
{
private:
  // subscriber
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size;

  // for sync checking
  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

  // for sync checking
  static void increment(int* value)
  {
    ++(*value);
  }

  void dataCb(const sensor_msgs::ImageConstPtr& l_image_msg,
              const sensor_msgs::ImageConstPtr& r_image_msg,
              const sensor_msgs::CameraInfoConstPtr& l_info_msg,
              const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {

    // For sync error checking
    ++all_received_;

    // call implementation
    imageCallback(l_image_msg, r_image_msg, l_info_msg, r_info_msg);
  }

 protected:

  /**
   * Constructor, subscribes to input topics using image transport and registers
   * callbacks.
   * \param transport The image transport to use
   */
  StereoProcessor(const std::string& transport) :
    left_received_(0), right_received_(0), left_info_received_(0), right_info_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");

    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";

    // Subscribe to four input topics.
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s",
        left_topic.c_str(), right_topic.c_str(),
        left_info_topic.c_str(), right_info_topic.c_str());

    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 3, transport);
    right_sub_.subscribe(it, right_topic, 3, transport);
    left_info_sub_.subscribe(nh, left_info_topic, 3);
    right_info_sub_.subscribe(nh, right_info_topic, 3);

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size, 5);
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
    exact_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4));
  }

  /**
   * Implement this method in sub-classes
   */
  virtual void imageCallback(const sensor_msgs::ImageConstPtr& l_image_msg,
                             const sensor_msgs::ImageConstPtr& r_image_msg,
                             const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                             const sensor_msgs::CameraInfoConstPtr& r_info_msg) = 0;

};

} // end of namespace

#endif
