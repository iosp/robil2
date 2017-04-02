#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <viso.h>

#include <viso_isl_ros/VisoInfo.h>

#include "stereo_processor.h"
#include "odometer_base.h"

namespace viso_isl_ros {

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


class StereoOdometer : public StereoProcessor, public OdometerBase
{
private:
  boost::shared_ptr<viso_isl::StereoVO> engine;
  viso_isl::params engine_params;
  
  ros::Publisher info_pub_;
  bool got_lost_;
  cv::Mat reference;

public:
  StereoOdometer(const std::string& transport) :
    StereoProcessor(transport), OdometerBase(),
    got_lost_(false)
  {
    ros::NodeHandle local_nh("~");
    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);
    reference = cv::Mat::eye(4, 4, cv::DataType<double>::type);
  }

protected:
  void init(const sensor_msgs::CameraInfoConstPtr& left_info_msg,
	    const sensor_msgs::CameraInfoConstPtr& right_info_msg)
  {
    int queue_size;
    bool approximate_sync;
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 5);
    local_nh.param("approximate_sync", approximate_sync, false);
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*left_info_msg, *right_info_msg);
    engine_params.base      = model.baseline();
    engine_params.calib.cu  = model.left().cx();
    engine_params.calib.cv  = model.left().cy();
    engine_params.calib.f   = model.left().fx();
    engine.reset(new viso_isl::StereoVO(engine_params));
    
    if (left_info_msg->header.frame_id != "")
      setSensorFrameId(left_info_msg->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& left_image_msg,
		     const sensor_msgs::ImageConstPtr& right_image_msg,
		     const sensor_msgs::CameraInfoConstPtr& left_info_msg,
		     const sensor_msgs::CameraInfoConstPtr& right_info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
    bool first_run = false;
    // create odometer if not exists
    if (!engine) {
      first_run = true;
      init(left_info_msg, right_info_msg);
    }

    cv_bridge::CvImageConstPtr left_cv_image, right_cv_image;
    left_cv_image = cv_bridge::toCvShare(left_image_msg, sensor_msgs::image_encodings::MONO8);
    right_cv_image = cv_bridge::toCvShare(right_image_msg, sensor_msgs::image_encodings::MONO8);

    ROS_ASSERT(left_image_msg->width == right_image_msg->width);
    ROS_ASSERT(left_image_msg->height == right_image_msg->height);

    bool success = engine->process_stereo_frame(left_cv_image->image, right_cv_image->image);
    if (first_run || got_lost_) {
      got_lost_ = false;
      if (first_run) {
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, left_image_msg->header.stamp);
      }
    } else {
#ifdef DEMO
      if (true || success) {
#else
      if (success) {
#endif
	cv::Mat motion = engine->getMotion();
	motion.inv();
        ROS_INFO_STREAM("libviso_isl returned the following motion:\n" << motion);
        reference = motion; // store last motion as reference
        tf::Matrix3x3 rot_mat(motion.at<double>(0,0), motion.at<double>(0,1), motion.at<double>(0,2),
			      motion.at<double>(1,0), motion.at<double>(1,1), motion.at<double>(1,2),
			      motion.at<double>(2,0), motion.at<double>(2,1), motion.at<double>(2,2));
        tf::Vector3 t(motion.at<double>(0,3), motion.at<double>(1,3), motion.at<double>(2,3));
        tf::Transform delta_transform(rot_mat, t);

        //setPoseCovariance(STANDARD_POSE_COVARIANCE);
        //setTwistCovariance(STANDARD_TWIST_COVARIANCE);
        integrateAndPublish(delta_transform, left_image_msg->header.stamp);
      } else {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, left_image_msg->header.stamp);

        ROS_DEBUG("Call to ::process() failed.");
        ROS_WARN_THROTTLE(10.0, "Visual Odometer got lost!");
        got_lost_ = true;
      }

      VisoInfo info_msg;
      info_msg.header.stamp = left_image_msg->header.stamp;
      info_msg.got_lost = !success;
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }
};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");
  
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso_isl_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }
  
  std::string transport = argc > 1 ? argv[1] : "raw";
  viso_isl_ros::StereoOdometer odometer(transport);
  ros::spin();
  return 0;
}
