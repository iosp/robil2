#include "localization.h"

localization::localization(ros::NodeHandle& n)
{
    ROS_INFO("Starting localization");
    /// Setting up publishers ///
    loc_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/LOC/Pose", 10);
    speed_pub = n.advertise<geometry_msgs::TwistStamped>("/LOC/Velocity", 10);
    /// Setup variables ///
    speed.header.frame_id = "ODOM";
    pose.header.frame_id = "ODOM";
    pose.header.stamp = ros::Time::now();
    /// Letting everyone know that LOC is ready ///
    ros::param::set("/LOC/Ready", 1);
}

localization::~localization(){ros::param::set("/LOC/Ready", 0);}
void localization::callback(const ImuConstPtr& imu,
              const NavSatFixConstPtr& gps,
              const NavSatFixConstPtr& speed_msg)
{
    /*
     * This function is the main callback function.
     * It synchronizes all of the message from the Shiffon
     * and publishes the necessary ROS messages and brodcats
     * the data as a TF node
     */
    if (initialGPS.latitude == 0 || dyn_conf.init)
    {
        ROS_INFO("LOC: Setting a new init position");
        initialGPS = *gps;
    }


    /// Handle the data ///
    this->handleSpeed(speed_msg, imu);
    this->handlePose(gps, imu);

    /// Set headers stamp ///(frame already taken care of in constructor)
    pose.header.stamp = speed.header.stamp = imu->header.stamp;

    /// publish the data ///
    this->publishersBroadcasters();
}

void localization::handleSpeed(const NavSatFixConstPtr& speed_msg, const ImuConstPtr& imu)
{
    /*
     * Set data of speed from messages
     **/
    /// Setting speed message. linear.z is the global speed
    speed.twist.linear.x = speed_msg->latitude;
    speed.twist.linear.y = -speed_msg->longitude;
    /// Add the global speed to z //
    speed.twist.linear.z = sqrt(speed.twist.linear.x * speed.twist.linear.x +
                                speed.twist.linear.y * speed.twist.linear.y);
    speed.twist.angular.x = imu->angular_velocity.x;
    speed.twist.angular.y = imu->angular_velocity.y;
    speed.twist.angular.z = imu->angular_velocity.z;
}

void localization::handlePose(const NavSatFixConstPtr& gps, const ImuConstPtr& imu)
{
    /*
     * Set data of pose from messages
     **/
    /// Handle GPS message
    double d = calcDistance(*gps,initialGPS);
    double theta = -calcBearing(initialGPS,*gps);
    double x = d * cos(theta);
    double y = d * sin(theta);
    pose.pose.pose.position.x = x;
    if (dyn_conf.right_hand_axes)
        pose.pose.pose.position.y = -y;
    else
        pose.pose.pose.position.y = y;

    pose.pose.pose.orientation = imu->orientation;
    /**
     * Calculating height from speed and pitch
     */

    double pitch = -this->quaternion_to_rpy(imu->orientation).y;
    double dt = (imu->header.stamp - pose.header.stamp).toSec();
    if (dyn_conf.height)
        pose.pose.pose.position.z = gps->altitude;
    else
        ; //pose.pose.pose.position.z += speed.twist.linear.z * tan(pitch) * dt;
}

void localization::publishersBroadcasters()
{
    /*
     * This function actually publishes the message and
     * creates a tf tranform between WORLD and ODOM
     */
    /// Publish the data
    speed_pub.publish(speed);
    loc_pub.publish(pose);
    /// make a static broadcaster ///
    static tf::TransformBroadcaster broadcaster;
    ///Setup a transform message out of pose ///
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pose.pose.position.x,
                                    pose.pose.pose.position.y,
                                    pose.pose.pose.position.z));
    transform.setRotation(tf::Quaternion(pose.pose.pose.orientation.x,
                                         pose.pose.pose.orientation.y,
                                         pose.pose.pose.orientation.z,
                                         pose.pose.pose.orientation.w));
    /// broadcast tf ///
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                   "WORLD", "ODOM"));
}

void localization::configCallback(loc::configConfig &config, uint32_t level)
{
    ROS_INFO("Reconfiguring LOC");
    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    dyn_conf = config;
}

geometry_msgs::Vector3 localization::quaternion_to_rpy(geometry_msgs::Quaternion q)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    geometry_msgs::Vector3 rpy;
    tf::Matrix3x3(quat).getRPY(rpy.x, rpy.y, rpy.z);
    return rpy;
}
