/*
 * Filename: CostMap.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 26, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
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


#include <navex/costmap/CostMap.h>
#include <assert.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


CostMap::CostMap(CostMapDataSourceBase* dataSource, double robotRadius, double inflationRadius, double inflation_siggma, double inflation_pow,
		double mapWidth, double mapHeight, double resolution,
		const string& frameId) {

	initializeCostMap(dataSource, NULL, robotRadius, inflationRadius, inflation_siggma, inflation_pow,
			mapWidth, mapHeight, resolution, frameId);
}

CostMap::CostMap(CostMapDataSourceBase* dataSource,
		IParametersProvider* parametersProvider, bool createMap) {
	BOOST_ASSERT_MSG(parametersProvider != NULL, "CostMap: Parameters provider is NULL");
	initializeCostMap(dataSource, parametersProvider, 0, 0, 0, 0, 0, 0, 0, "", createMap);
}

CostMap::CostMap(CostMapDataSourceBase* dataSource, double robotRadius,
		double inflationRadius, double inflation_siggma, double inflation_pow
		) {
	initializeCostMap(dataSource, NULL, robotRadius, inflationRadius, inflation_siggma, inflation_pow,
			-1, -1, -1, "");
}


CostMap::~CostMap() {
	if (dataSource_ != NULL) {
		delete dataSource_;
		dataSource_ = NULL;
	}

	if (parametersProvider_ != NULL) {
		delete parametersProvider_;
		parametersProvider_ = NULL;
	}
}

void CostMap::initializeCostMap(CostMapDataSourceBase* dataSource,
		IParametersProvider* parametersProvider, double robotRadius,
		double inflationRadius, double inflation_siggma, double inflation_pow,
		double mapWidth, double mapHeight, double resolution,
		string frameId, bool allocateMapArray) {


	dataSource_ = dataSource;
	parametersProvider_ = parametersProvider;

	robotRadius_ = robotRadius;
	inflationRadius_ = inflationRadius;

	inflation_template__inflation_sigmma_ = inflation_siggma;
	inflation_template__inflation_pow_ = inflation_pow;

	/**
	 * If parametersProvider presents, use it
	 * to fetch parameters
	 */
	if (parametersProvider != NULL) {

		// Subscribes to update parameters event
		parametersProvider->updateParametersCallback =
				boost::bind(&CostMap::updateParameters, this);

		robotRadius_ = parametersProvider->getRobotRadius();
		inflationRadius_ = parametersProvider->getInflationRadius();
		inflation_template__inflation_sigmma_ = parametersProvider->getInflationSigmma();
		inflation_template__inflation_pow_ = parametersProvider->getInflationPow();

		mapWidth = parametersProvider->getMapWidth();
		mapHeight = parametersProvider->getMapHeight();
		resolution = parametersProvider->getMapResolution();
		frameId = parametersProvider->getMapFrameId();
	}


//	dataSource_ = dataSource;
//	parametersProvider_ = parametersProvider;
//	robotRadius_ = robotRadius;
//	inflationRadius_ = inflationRadius;
//
//	inflation_template__inflation_sigmma_ = inflation_siggma;
//	inflation_template__inflation_pow_ = inflation_pow;

	/**
	 * Creates map array
	 */

	if (allocateMapArray)
		createOccupancyGrid(mapWidth, mapHeight, resolution,
				-mapWidth / 2.0, -mapHeight / 2.0, frameId);
	else
		createOccupancyGrid(0.0, 0.0, 0.0, 0.0, 0.0, "");

	if (dataSource_ != NULL) {
		/**
		 * Subscribe to data source's callbacks
		 */
		dataSource_->clearMapCallback =
				boost::bind(&CostMap::clearMapCallback, this);
		dataSource_->updatePointsCallback =
				boost::bind(&CostMap::updatePoints, this, _1);
		dataSource_->createOccupancyGridCallback =
				boost::bind(&CostMap::createOccupancyGrid, this, _1, _2, _3, _4, _5, _6);
	}

	// TODO add map updated event
//	raiseMapUpdated();

	printSummary();
}

void CostMap::printSummary() const {
	ROS_INFO("CostMap summary:");

	ROS_INFO(" Data source: %s", (dataSource_ == NULL ?
			"NONE" : dataSource_->getName().c_str()));

	ROS_INFO("       Frame: %s",
				occupancyGrid_->header.frame_id.c_str());

	ROS_INFO("       Width: %fm (%ipx)",
			occupancyGrid_->info.width * occupancyGrid_->info.resolution,
			occupancyGrid_->info.width);

	ROS_INFO("      Height: %fm (%ipx)",
			occupancyGrid_->info.height * occupancyGrid_->info.resolution,
			occupancyGrid_->info.height);

	ROS_INFO("  Resolution: %fm/px",
			occupancyGrid_->info.resolution);

	ROS_INFO("Robot radius: %fm (%ipx)",
				robotRadius_,
				(int)(robotRadius_ / occupancyGrid_->info.resolution));

	ROS_INFO("   Inflation: %fm (%ipx)",
			inflationRadius_,
			(int)(inflationRadius_ / occupancyGrid_->info.resolution));

	ROS_INFO("    Origin x: %fm (%ipx)",
			occupancyGrid_->info.origin.position.x,
			(int)(occupancyGrid_->info.origin.position.x / occupancyGrid_->info.resolution));

	ROS_INFO("    Origin y: %fm (%ipx)",
			occupancyGrid_->info.origin.position.y,
			(int)(occupancyGrid_->info.origin.position.y / occupancyGrid_->info.resolution));
}

void CostMap::createOccupancyGrid(double mapWidth,
		double mapHeight, double resolution, double originX, double originY,
		const string& frameId) {

	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	occupancyGrid_ = nav_msgs::OccupancyGrid::Ptr(
			new nav_msgs::OccupancyGrid());

	if (mapHeight <= 0.0 || mapWidth <= 0.0) {
		// Map is not initialized, should be filled later
		return;
	}

	occupancyGrid_->header.frame_id = frameId;
	occupancyGrid_->info.width = mapWidth / resolution;
	occupancyGrid_->info.height = mapHeight / resolution;
	occupancyGrid_->info.resolution = resolution;
	occupancyGrid_->info.origin.position.x = originX;
	occupancyGrid_->info.origin.position.y = originY;
	occupancyGrid_->info.origin.orientation.w = 1;

	if (parametersProvider_ != NULL) {
		parametersProvider_->setMapWidth(mapWidth);
		parametersProvider_->setMapHeight(mapHeight);
		parametersProvider_->setMapResolution(resolution);
		parametersProvider_->setFrameId(frameId);
	}

	occupancyGrid_->data.resize(
			occupancyGrid_->info.width * occupancyGrid_->info.height);

	cvMatrix_ = cv::Mat(
			occupancyGrid_->info.height, occupancyGrid_->info.width,
			CV_8SC1, occupancyGrid_->data.data());

}

CostMapCell::CellType CostMap::getCellValue(uint32_t x, uint32_t y) const {
	cv::Point pixel(x, y);

	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	if (isInBounds(cv::Point(x, y)))
		return cvMatrix_.at<char>(pixel);

	/**
	 * Out of map points considered as unknown
	 */
	return CostMapCell::CELL_UNKNOWN;
}

CostMapCell::CellType CostMap::getCellValue(const geometry_msgs::Pose& pose) const {
	cv::Point pixel = localCoordinatesToPixel(pose.position.x, pose.position.y);

	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	if (isInBounds(pixel))
		return cvMatrix_.at<char>(pixel);

	/**
	 * Out of map points considered as unknown
	 */
	return CostMapCell::CELL_UNKNOWN;
}

CostMapCell::CellType CostMap::getCellValue(const geometry_msgs::PoseStamped& poseStamped) const {

	geometry_msgs::PoseStamped localPoseStamped;

	try {
		tfListener_.transformPose(getFrameId(),
					poseStamped, localPoseStamped);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform pose: \n%s", exception.what());
		return CostMapCell::CELL_UNKNOWN;
	}

	return getCellValue(localPoseStamped.pose);
}

CostMapCell::CellType CostMap::getCellValue(double x, double y,
		const string& frameId, const ros::Time& stamp) const {
	geometry_msgs::PoseStamped poseStamped;

	poseStamped.header.frame_id = frameId;
	poseStamped.header.stamp = stamp;

	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	poseStamped.pose.orientation.w = 1;

	return getCellValue(poseStamped);
}

cv::Point CostMap::poseToPixel(
		const geometry_msgs::PoseStamped& pose) const {
	geometry_msgs::PoseStamped localPoseStamped;

	try {
		tfListener_.waitForTransform(getFrameId(), pose.header.frame_id,
				pose.header.stamp, ros::Duration(5.0));
		tfListener_.transformPose(getFrameId(),
				pose, localPoseStamped);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform pose: \n%s", exception.what());
		throw;
	}

	return localCoordinatesToPixel(localPoseStamped.pose.position.x, localPoseStamped.pose.position.y);
}


void CostMap::clearCell(const cv::Point& pixel, double radius) {
	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	cv::circle(cvMatrix_, pixel, radius / occupancyGrid_->info.resolution, CostMapCell::CELL_FREE, -1);
}

void CostMap::merge(const CostMap& costmap, bool ignoreFree) {
	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	ros::Time poseTime = ros::Time::now();

	for(int y = 0; y < costmap.getCvMatrix().rows; y++)
	    for(int x = 0; x < costmap.getCvMatrix().cols; x++)
	    {
	    	geometry_msgs::PoseStamped pose = costmap.pixelToPose(cv::Point(x, y));
	    	pose.header.stamp = poseTime;
	    	cv::Point localPixel = this->poseToPixel(pose);

	    	if (isInBounds(localPixel)) {
				CostMapCell::CellType value = costmap.getCvMatrix().at<CostMapCell::CellType>(y, x);

				if (!ignoreFree || value >= CostMapCell::CELL_BLOCKED) {

					this->setCellValue(localPixel, value);

					// Nasty workaround
					// To prevent infinite replanning
					this->setCellValue(localPixel + cv::Point(1, 1), value);
					this->setCellValue(localPixel + cv::Point(-1, 1), value);
					this->setCellValue(localPixel + cv::Point(1, -1), value);
					this->setCellValue(localPixel + cv::Point(-1, -1), value);
				}
	    	}
	    }
}

geometry_msgs::PoseStamped CostMap::pixelToPose(cv::Point pixel) const {
	return pixelToPose((double)pixel.x, (double)pixel.y);
}
geometry_msgs::PoseStamped CostMap::pixelToPose(cv::Point2d pixel) const {
	return pixelToPose(pixel.x, pixel.y);
}

geometry_msgs::PoseStamped CostMap::pixelToPose(uint32_t x, uint32_t y) const {
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = getFrameId();
	pose.header.stamp = ros::Time::now();

	pose.pose.position.x = occupancyGrid_->info.resolution * x +
			occupancyGrid_->info.origin.position.x;
	pose.pose.position.y = occupancyGrid_->info.resolution * y +
			occupancyGrid_->info.origin.position.y;

	ROS_INFO_STREAM("COST MAP 5: "<<pose.pose.position.x<<","<<pose.pose.position.y);
	pose.pose.orientation.w = 1;

	return pose;
}

geometry_msgs::PoseStamped CostMap::pixelToPose(double x, double y) const {
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = getFrameId();
	pose.header.stamp = ros::Time::now();

	pose.pose.position.x = occupancyGrid_->info.resolution * x +
			occupancyGrid_->info.origin.position.x;
	pose.pose.position.y = occupancyGrid_->info.resolution * y +
			occupancyGrid_->info.origin.position.y;

	pose.pose.orientation.w = 1;

	return pose;
}


cv::Mat& CostMap::get_template_for_inflation(double maybeBlockedInflation, double blockedInflation)
{
	typedef double T_RobotRadius;
	typedef double T_inflationRadius;
	typedef double T_MapResolution;
	typedef double T_inflationSigma;
	typedef double T_inflationPow;

	typedef std::map<T_RobotRadius,
			 std::map<T_inflationRadius,
			   std::map<T_MapResolution,
				 std::map<T_inflationSigma,
				   std::map<T_inflationPow,
					 cv::Mat
				   >
				 >
			   >
			 >
		   >
		   Cach;

	static Cach inflation_template_cach;

	cv::Mat& inflation_template_ = inflation_template_cach
						[robotRadius_]
						[inflationRadius_]
						[occupancyGrid_->info.resolution]
						[inflation_template__inflation_sigmma_]
						[inflation_template__inflation_pow_];
	if( inflation_template_.empty() )
	{
		create_template_for_inflation(maybeBlockedInflation, blockedInflation).copyTo(inflation_template_);
	}
	return inflation_template_;
}
cv::Mat CostMap::create_template_for_inflation(double maybeBlockedInflation, double blockedInflation)
{
	ROS_WARN_STREAM("COST MAP: CACH MAP: "<<robotRadius_ <<", "<<inflationRadius_ <<", "<< occupancyGrid_->info.resolution<<", "<< inflation_template__inflation_sigmma_<<", "<<inflation_template__inflation_pow_);

	class gaussean{
		double m,s,D,E;
	public:
		gaussean(double m, double s):m(m),s(s),D(2*s*s),E(sqrt(D*M_PI)){}
		double operator()(double x)const{ double n = x-m; return exp(-(n*n)/D)/E; }
	};
	class norm_gaussean{
		double m,s,b;
	public:
		norm_gaussean(double m, double s):m(m),s(s),b(gaussean(m,s)(m)){}
		double operator()(double x)const{ return gaussean(m,s)(x)/b; }
	};

	const int radius = (int)round(maybeBlockedInflation*1.);
	cv::Mat inflation_template_ = cv::Mat(radius*2+1, radius*2+1, CV_8SC1, cv::Scalar::all(0.0));
	double max_dist = radius;

	double icx = inflation_template_.cols/2, icy = inflation_template_.rows/2;

	double cb = inflation_template__inflation_sigmma_, cc = inflation_template__inflation_pow_;
	double r =blockedInflation/radius;
	for(int y=0;y<inflation_template_.rows;y++)for(int x=0;x<inflation_template_.cols;x++)
	{
		double dist = hypot(x-icx,y-icy);
		if(dist > radius)
		{
			inflation_template_.at<char>(y,x) = -1;
			continue;
		}
		if(dist <= blockedInflation)
		{
			inflation_template_.at<char>(y,x) = 100;
			continue;
		}
		double value = dist/max_dist;
		double _r = value;
		value = fmin(1, value);
		value = pow(norm_gaussean(r, cb)(value),cc);

		if(value < 0.01)
			inflation_template_.at<char>(y,x) = -1;
		else
			inflation_template_.at<char>(y,x) = round(value*100);
	}

	if(false)
	{
		static long fname_counter=0;fname_counter++;
		std::stringstream fname; fname <<"/tmp/MAP/"<<"inftmp_"<<robotRadius_ <<"_"<<inflationRadius_ <<"_"<< occupancyGrid_->info.resolution<<"_"<< inflation_template__inflation_sigmma_<<"_"<<inflation_template__inflation_pow_<<"-"<<fname_counter<<".png";
		cv::imwrite(fname.str(), inflation_template_);
	}

	ROS_WARN_STREAM("COST MAP: CACH MAP: DONE");

	if(false)
	{
		const int radius = (int)round(maybeBlockedInflation*1.);
		cv::Mat colored (inflation_template_.size(), CV_8UC3, cv::Scalar::all(255));
		cv::Point2d h01(0,0), h05(0,0), h03(0,0), h07(0,0), h09(0,0);
		for(int y=0;y<colored.rows;y++)for(int x=0;x<colored.cols;x++)
		{
			char vv = inflation_template_.at<char>(y,x);
			double v = (vv+1)/101.;
			colored.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0)*v+ (1-v)*cv::Vec3b(255,255,255);
			if( fabs(vv - 10) < 1E-2 ){ h01 = cv::Point2d(x,y) - (cv::Point2d(colored.cols,colored.rows)*0.5); }
			if( fabs(vv - 30) < 1E-2 ){ h03 = cv::Point2d(x,y) - (cv::Point2d(colored.cols,colored.rows)*0.5); }
			if( fabs(vv - 50) < 1E-2 ){ h05 = cv::Point2d(x,y) - (cv::Point2d(colored.cols,colored.rows)*0.5); }
			if( fabs(vv - 70) < 1E-2 ){ h07 = cv::Point2d(x,y) - (cv::Point2d(colored.cols,colored.rows)*0.5); }
			if( fabs(vv - 90) < 1E-2 ){ h09 = cv::Point2d(x,y) - (cv::Point2d(colored.cols,colored.rows)*0.5); }
		}

		cv::circle(colored, cv::Point(colored.cols/2,colored.rows/2), blockedInflation, cv::Scalar(255,0,255), 0);
#define DDD(N,P)	cv::circle(colored, cv::Point(colored.cols/2,colored.rows/2), hypot(h##N.x,h##N.y), cv::Scalar(255*P,0,255), 0);
		DDD(01,.1)
		DDD(03,.3)
		DDD(05,.5)
		DDD(07,.7)
		DDD(09,.9)
#undef DDD
		cv::circle(colored, cv::Point(colored.cols/2,colored.rows/2), radius, cv::Scalar(0,0,255), 0);
		std::stringstream fname; fname <<"/tmp/MAP/"<<"inftmp_"<< occupancyGrid_->info.resolution<<".png";
		cv::imwrite(fname.str(),colored);
	}

	return inflation_template_;
}

void CostMap::apply_inflation(const cv::Mat& inflation_template_, cv::Mat& cvMatrix_)
{
	if(inflation_template_.empty())
		ROS_ERROR("COST MAP : INFLATION TEMPLATE IS EMPTY");

	for (size_t i = 0; i < points_.size() ; ++i) {
		cv::Point p = cv::Point(points_[i].x, points_[i].y);

		if (not isInBounds(p)) continue;

		apply_inflation(inflation_template_, cvMatrix_, p);
	}
}

void CostMap::apply_inflation(const cv::Mat& inflation_template_, cv::Mat& cvMatrix_, const cv::Point& p)
{
	//---------- OTHER VERSION : NEED TESTING ---------------------
#ifndef OTHER_VERSION_OF_MERGE
	using namespace cv;
	struct Rect2
	{
		Point2d lt, rb;
		Rect2(double l, double t, double w, double h):lt(l,t),rb(l+w-1,t+h-1){}
		Rect2(Point2d _lt, Point2d _rb):lt(_lt), rb(_rb){}
		Point2d center()const{ return Point( (lt.x+rb.x)/2., (lt.y+rb.y)/2. ); }
		Size size()const{ return Size(rb.x-lt.x+1,rb.y-lt.y+1); }
		Rect2 translate(const Point2d& p)const{ return Rect2(lt+p, rb+p); }
		Rect2 intersect(const Rect2& r)const{ return Rect2(
				Point(fmax(lt.x, r.lt.x), fmax(lt.y, r.lt.y)),
				Point(fmin(rb.x, r.rb.x), fmin(rb.y, r.rb.y))); }
	};

	struct Merger
	{
		bool in_range(const Point2d p, const Rect2& r)const
		{
			return r.lt.x<=p.x and p.x<=r.rb.x and r.lt.y<=p.y and p.y<=r.rb.y;
		}
		bool in_range(const Rect2& a, const Rect2& r)const
		{
			return in_range(a.lt, r) and in_range(a.rb, r);
		}
		bool operator()(const Mat& imgTemplate, const Mat& imgTarget, const Point& p)const
		{
			Point2d location(p.x, p.y);
			Rect2 m1r(0,0,imgTemplate.cols, imgTemplate.rows);
			Rect2 m2r(0,0,imgTarget.cols, imgTarget.rows);
			Point2d offset = location - m1r.center();

			Rect2 m1_in_m2r = m1r.translate(offset);
			Rect2 intersect_in_m2r = m2r.intersect(m1_in_m2r);
			Rect2 intersect_in_m1r = intersect_in_m2r.translate(offset*-1.0);

			assert( in_range(intersect_in_m1r, m1r) );
			Mat m1s = imgTemplate.colRange(intersect_in_m1r.lt.x, intersect_in_m1r.rb.x).rowRange(intersect_in_m1r.lt.y, intersect_in_m1r.rb.y);

			assert( in_range(intersect_in_m2r, m2r) );
			Mat m2s = imgTarget.colRange(intersect_in_m2r.lt.x, intersect_in_m2r.rb.x).rowRange(intersect_in_m2r.lt.y, intersect_in_m2r.rb.y);

			m2s = cv::max(m1s, m2s);
		}
	};

	Merger m;
	try{
		m(inflation_template_, cvMatrix_, p);
	}catch(...){ ROS_WARN("COST MAP: Cann't update with this point"); }
#else
	double icx = inflation_template_.cols/2, icy = inflation_template_.rows/2;

	int l,r,t,b;
	l = fmax(0, p.x-icx);
	r = fmin(cvMatrix_.cols-1, p.x+icx);
	t = fmax(0, p.y-icy);
	b = fmin(cvMatrix_.rows-1, p.y+icy);

	if( not( 0<=l and 0<=r and 0<=t and 0<=b and l<cvMatrix_.cols and r<cvMatrix_.cols and t<cvMatrix_.rows and b<cvMatrix_.rows ) )
	{
		//NOTE: This is happen in first creation of map after changing of parameters (inflation properties) in dynamic reconfigure.
		ROS_WARN("COST MAP: Cann't update with this point");
		return;
	}
	cv::Mat target = cvMatrix_.colRange(l, r).rowRange(t,b);

	int tl, tr, tt, tb;
	tl = fmax(0, icx-(p.x-l));
	tr = fmin(inflation_template_.cols-1, icx+(r-p.x));
	tt = fmax(0, icy-(p.y-t));
	tb = fmin(inflation_template_.rows-1, icy+(b-p.y));

	assert( 0<=tl and 0<=tr and 0<=tt and 0<=tb and tl<inflation_template_.cols and tr<inflation_template_.cols and tt<inflation_template_.rows and tb<inflation_template_.rows );
	cv::Mat source = inflation_template_.colRange(tl, tr).rowRange(tt,tb);

	target = cv::max( target, source);
#endif

}


void CostMap::updatePoints(const CostMapDataContainer& dataContainer) {

	boost::recursive_mutex::scoped_lock lock(readWriteMutex_);

	points_ = dataContainer;

	if (!points_.isPixelCoordinates()) {

		/*
		 * Points are not pixel coordinates
		 */
		if (dataContainer.getFrameId().size() > 0 &&
				dataContainer.getFrameId() != this->getFrameId()) {

			tf::StampedTransform dataSourceToMapTransform;
			tf::Vector3 localMapCoordinates;

			/*
			 * Points must be transformed to costmap's coordinates frame
			 */
			try {
				if (tfListener_.waitForTransform(getFrameId(), points_.getFrameId(),
						points_.getStamp(), ros::Duration(5.0))) {
					tfListener_.lookupTransform(getFrameId(),
							points_.getFrameId(), points_.getStamp(), dataSourceToMapTransform);
				} else {
					ROS_ERROR("Failed to transform point, transform not found\n "
							"%s -> %s", points_.getFrameId().c_str(),
							getFrameId().c_str());

				}
			}
			catch (tf::TransformException& exception) {
				ROS_ERROR("Failed to transform point: \n%s", exception.what());
				return;
			}

			cv::Point pixel;
			for (size_t i = 0; i < points_.size(); ++i) {
				PointData& point = points_[i];

				localMapCoordinates = dataSourceToMapTransform *
						tf::Vector3(point.x, point.y, point.z);

				pixel = localCoordinatesToPixel(localMapCoordinates);

				point.x = pixel.x;
				point.y = pixel.y;
				point.z = 0;
			}

		} else {
			/*
			 * Points in map's coordinates frame, translate to pixels
			 */
			cv::Point pixel;
			for (size_t i = 0; i < points_.size(); ++i) {
				PointData& point = points_[i];
				pixel = localCoordinatesToPixel(point.x, point.y);

				point.x = pixel.x;
				point.y = pixel.y;
				point.z = 0;
			}
		}
	}

	const double maybeBlockedInflation =
			(robotRadius_ + inflationRadius_) / occupancyGrid_->info.resolution;

	const double blockedInflation =
			(robotRadius_) / occupancyGrid_->info.resolution;

	cv::Mat inflation_template_ = get_template_for_inflation(maybeBlockedInflation, blockedInflation);
	apply_inflation(inflation_template_, cvMatrix_);

	occupancyGrid_->header.stamp = points_.getStamp();
}

void CostMap::updateParameters() {

	if (parametersProvider_ == NULL)
		return;

	robotRadius_ = parametersProvider_->getRobotRadius();
	inflationRadius_ = parametersProvider_->getInflationRadius();
	inflation_template__inflation_sigmma_ = parametersProvider_->getInflationSigmma();
	inflation_template__inflation_pow_ = parametersProvider_->getInflationPow();

	double mapWidth = parametersProvider_->getMapWidth();
	double mapHeight = parametersProvider_->getMapHeight();
	double resolution = parametersProvider_->getMapResolution();
	string frameId = parametersProvider_->getMapFrameId();

	bool mapSizeOrResolutionChanged = !occupancyGrid_ ||
			(mapWidth != occupancyGrid_->info.width * occupancyGrid_->info.resolution) ||
			(mapHeight != occupancyGrid_->info.height * occupancyGrid_->info.resolution) ||
			(resolution != occupancyGrid_->info.resolution);

	if (mapSizeOrResolutionChanged) {

		double originX = -mapWidth / 2.0;
		double originY = -mapHeight / 2.0;

		if (occupancyGrid_) {
			originX = occupancyGrid_->info.origin.position.x;
			originY = occupancyGrid_->info.origin.position.y;
		}

		createOccupancyGrid(mapWidth, mapHeight, resolution,
				originX, originY, frameId);

		this->updatePoints(this->points_);

	} else if (frameId != getFrameId())

		if (!occupancyGrid_)
			return;

		occupancyGrid_->header.frame_id = frameId;
}
