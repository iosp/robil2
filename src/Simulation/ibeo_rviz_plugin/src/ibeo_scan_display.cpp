/*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/time.h>

#include <laser_geometry/laser_geometry.h>

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"
#include "gazebo_ibeo_gpu/MultiLaserScan.h"
#include "ibeo_scan_display.h"
#include "sensor_msgs/LaserScan.h"
#include <string.h>
#include <iostream>
#include <fstream>

namespace ibeo_rviz_plugin
{

IBEOScanDisplay::IBEOScanDisplay()
  : point_cloud_common_( new rviz::PointCloudCommon( this ))
  , projector_( new laser_geometry::LaserProjection() )
{
  queue_size_property_ = new rviz::IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming LaserScan message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your LaserScan data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance. Use that for processing incoming messages.
  update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );
}

IBEOScanDisplay::~IBEOScanDisplay()
{
  delete point_cloud_common_;
  delete projector_;
}

void IBEOScanDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize( context_, scene_node_ );
}

void IBEOScanDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void IBEOScanDisplay::processMessage( const gazebo_ibeo_gpu::MultiLaserScanConstPtr& scan )
{
  sensor_msgs::PointCloudPtr cloud( new sensor_msgs::PointCloud );

  std::string frame_id = "ibeo";//scan->header.frame_id;

  // Compute tolerance necessary for this scan
  ros::Duration tolerance( scan->time_increment*(scan->ranges_b1.size()+
						scan->ranges_b2.size()+
						scan->ranges_t1.size()+
						scan->ranges_t2.size()));
  if (tolerance > filter_tolerance_)
  {
    filter_tolerance_ = tolerance;
    tf_filter_->setTolerance(filter_tolerance_);
  }

  sensor_msgs::LaserScan scan1;
  scan1.header.frame_id=frame_id;
  scan1.header.seq=scan->header.seq;
  scan1.header.stamp=scan->header.stamp;
  scan1.angle_increment=scan->angle_increment;
  scan1.angle_max=scan->angle_max_b;
  scan1.angle_min=scan->angle_min_b;
  scan1.intensities=scan->intensities;
  scan1.range_max=scan->range_max;
  scan1.range_min=scan->range_min;
  scan1.ranges=scan->ranges_b1;
  scan1.scan_time=scan->scan_time;
  scan1.time_increment=scan->time_increment;


  try
  {
    projector_->transformLaserScanToPointCloud( fixed_frame_.toStdString(), scan1, *cloud, *context_->getTFClient(),
                                                laser_geometry::channel_option::Intensity );
  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG( "LaserScan [%s]: failed to transform scan: %s. This message should not repeat (tolerance should now be set on our tf::MessageFilter).",
               qPrintable( getName() ), e.what() );
    return;
  }

  point_cloud_common_->addMessage( cloud );

 sensor_msgs::LaserScan scan2;
  scan2.header.frame_id=frame_id;
  scan2.header.seq=scan->header.seq;
  scan2.header.stamp=scan->header.stamp;
  scan2.angle_increment=scan->angle_increment;
  scan2.angle_max=scan->angle_max_b;
  scan2.angle_min=scan->angle_min_b;
  scan2.intensities=scan->intensities;
  scan2.range_max=scan->range_max;
  scan2.range_min=scan->range_min;
  scan2.ranges=scan->ranges_b2;
  scan2.scan_time=scan->scan_time;
  scan2.time_increment=scan->time_increment;

  try
  {
    projector_->transformLaserScanToPointCloud( fixed_frame_.toStdString(), scan2, *cloud, *context_->getTFClient(),
                                                laser_geometry::channel_option::Intensity );
  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG( "LaserScan [%s]: failed to transform scan: %s. This message should not repeat (tolerance should now be set on our tf::MessageFilter).",
               qPrintable( getName() ), e.what() );
    return;
  }

  point_cloud_common_->addMessage( cloud );

  sensor_msgs::LaserScan scan3;
  scan3.header.frame_id=frame_id;
  scan3.header.seq=scan->header.seq;
  scan3.header.stamp=scan->header.stamp;
  scan3.angle_increment=scan->angle_increment;
  scan3.angle_max=scan->angle_max_t;
  scan3.angle_min=scan->angle_min_t;
  scan3.intensities=scan->intensities;
  scan3.range_max=scan->range_max;
  scan3.range_min=scan->range_min;
  scan3.ranges=scan->ranges_t1;
  scan3.scan_time=scan->scan_time;
  scan3.time_increment=scan->time_increment;

  try
  {
    projector_->transformLaserScanToPointCloud( fixed_frame_.toStdString(), scan3, *cloud, *context_->getTFClient(),
                                                laser_geometry::channel_option::Intensity );
  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG( "LaserScan [%s]: failed to transform scan: %s. This message should not repeat (tolerance should now be set on our tf::MessageFilter).",
               qPrintable( getName() ), e.what() );
    return;
  }

  point_cloud_common_->addMessage( cloud );

  sensor_msgs::LaserScan scan4;
  scan4.header.frame_id=frame_id;
  scan4.header.seq=scan->header.seq;
  scan4.header.stamp=scan->header.stamp;
  scan4.angle_increment=scan->angle_increment;
  scan4.angle_max=scan->angle_max_t;
  scan4.angle_min=scan->angle_min_t;
  scan4.intensities=scan->intensities;
  scan4.range_max=scan->range_max;
  scan4.range_min=scan->range_min;
  scan4.ranges=scan->ranges_t2;
  scan4.scan_time=scan->scan_time;
  scan4.time_increment=scan->time_increment;

  try
  {
    projector_->transformLaserScanToPointCloud( fixed_frame_.toStdString(), scan4, *cloud, *context_->getTFClient(),
                                                laser_geometry::channel_option::Intensity );
  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG( "LaserScan [%s]: failed to transform scan: %s. This message should not repeat (tolerance should now be set on our tf::MessageFilter).",
               qPrintable( getName() ), e.what() );
    return;
  }

  point_cloud_common_->addMessage( cloud );
}

void IBEOScanDisplay::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );
}

void IBEOScanDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ibeo_rviz_plugin::IBEOScanDisplay, rviz::Display )
