/*
 * Filename: PointCloudDataSource.h
 *   Author: Igor Makhtes
 *     Date: Jan 4, 2016
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDLASERCLEANERDATASOURCE_H_
#define INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDLASERCLEANERDATASOURCE_H_


#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <navex/costmap/datasource/CostMapDataSourceBase.h>
#include <navex/costmap/parameters/PCDataDynamicConfigParameterProvider.h>

using namespace std;


/**
 * Uses @see sensor_msgs::PointCloud2 as data source for @see CostMap
 */
template <class PointT>
class PointCloudReconfiguredDataSource : public CostMapDataSourceBase {

public:

	PointCloudReconfiguredDataSource(const string& topic, PCDataDynamicConfigParameterProvider * config,double update_rate=30.0,
			ros::NodeHandle nodeHandle = ros::NodeHandle())
		:nodeHandle_(nodeHandle),config_(config),update_rate_(update_rate),first_message_arrived_(false)
			,decay_thread_(boost::bind(&PointCloudReconfiguredDataSource<PointT>::decayThread,this)),
			update_thread_(boost::bind(&PointCloudReconfiguredDataSource<PointT>::updateThread,this)){

		cloudSubscriber_ = nodeHandle.subscribe("/map_cloud", 10, &PointCloudReconfiguredDataSource::pointCloudCallback, this);
		laserSubscriber_ = nodeHandle.subscribe("/costmap_clear_fake_scan", 10, &PointCloudReconfiguredDataSource::laserCallback, this);
//		if(update_rate > 0.0001)
//			update_thread_ = boost::thread(boost::bind(&PointCloudReconfiguredDataSource<PointT>::updateThread,this));
	}

	virtual ~PointCloudReconfiguredDataSource() {
		decay_thread_.interrupt();
		decay_thread_.join();
		update_thread_.interrupt();
		update_thread_.join();
		if(config_ != NULL)
		{
			delete config_;
			config_ = NULL;
		}
	}

public:

	virtual inline string getName() const {
		return "PointCloud";
	}

private:
	double update_rate_;
	boost::mutex update_mutex_;

	bool first_message_arrived_;

	boost::posix_time::ptime last_callback_;

	ros::Subscriber cloudSubscriber_;
	ros::Subscriber laserSubscriber_;

	ros::NodeHandle nodeHandle_;

	PCDataDynamicConfigParameterProvider * config_;

	tf::TransformListener tfListener_;

	CostMapDataContainer points_;


	boost::thread decay_thread_;
	boost::thread update_thread_;

private:

	void decayThread()
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		while(ros::ok())
		{
			boost::this_thread::interruption_point();
			double decay_time = config_->getDecayTime();
			if(decay_time > 0.0001)
			{
				ros::Rate r(1./decay_time);
				CostMapDataSourceBase::clearMap();
				r.sleep();
			}
			else
				boost::this_thread::sleep(boost::posix_time::milliseconds(200));
		}
	}

	void updateThread()
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

		if(update_rate_ < 0.0001)
			return;

		ros::Rate r(update_rate_);
		while(ros::ok())
		{
			{
				boost::mutex::scoped_lock l(update_mutex_);
				boost::this_thread::interruption_point();
				CostMapDataSourceBase::clearMap();
				if(first_message_arrived_)
				{
					points_.setStamp(ros::Time(0));
					updatePoints(points_);
				}
			}
			r.sleep();
		}
	}

	void laserCallback(const sensor_msgs::LaserScanConstPtr & scan)
	{
//		CostMapDataSourceBase::clearMap();
	}

	void pointCloudCallback(const sensor_msgs::PointCloud::Ptr & c0) {
		sensor_msgs::PointCloud c1;

		try
		{
			tfListener_.transformPointCloud(config_->getDataFrameId(),*c0,c1);
		}
		catch(...)
		{
			return;
		}

		sensor_msgs::PointCloud2 c2;
		sensor_msgs::convertPointCloudToPointCloud2(c1,c2);

		pcl::PointCloud<PointT> cloud;

		pcl::fromROSMsg(c2,cloud);

		double decay_time = config_->getDecayTime();

		boost::mutex::scoped_lock l(update_mutex_);

		points_ = CostMapDataContainer(false, config_->getDataFrameId(), pcl_conversions::fromPCL(cloud.header).stamp);

		// 5-15 => 0.0-1.0
		double min = 5.0;
		double max = 15.0;
		double nmin = 0.0;
		double nmax = 1.0;
		for (int i = 0; i < cloud.points.size(); ++i) {
			const double& x = cloud.points[i].x;
			const double& y = cloud.points[i].y;
			const double& z = cloud.points[i].z;

			points_.push_back(PointData(x,y, 0, CostMapCell::CELL_BLOCKED));
		}

		if(update_rate_ < 0.0001)
		{
			if(decay_time <= 0.0001)
				CostMapDataSourceBase::clearMap();

			updatePoints(points_);
		}

		first_message_arrived_ = true;
	}

};

#endif /* INCLUDE_NAVEX_COSTMAP_DATASOURCE_POINTCLOUDLASERCLEANERDATASOURCE_H_ */
