/*
 * Filename: Trajectory.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
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


#include <navex/trajectory/Trajectory.h>


Trajectory::Trajectory(const IMotionModel* motionModel, double weight)
	: weight_(weight), motionModel_(const_cast<IMotionModel*>(motionModel)),
	  overrideInPlace_(false), overrideInPlaceValue_(false) {
	path_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
}

void Trajectory::addPosition(double x, double y) {
	geometry_msgs::PoseStamped newPose;

	newPose.header.stamp = ros::Time::now();
	newPose.pose.orientation.w = 1;
	newPose.pose.position.x = x;
	newPose.pose.position.y = y;

	path_->poses.push_back(newPose);
}

void Trajectory::addPosition(const tf::Vector3& position,
		const tf::Quaternion& orientation)
{
	geometry_msgs::PoseStamped newPose;

	// newPose.header.frame_id = ""; // set in getPath()
	newPose.header.stamp = ros::Time::now();

	newPose.pose.position.x = position.x();
	newPose.pose.position.y = position.y();
	newPose.pose.position.z = position.z();

	newPose.pose.orientation.x = orientation.x();
	newPose.pose.orientation.y = orientation.y();
	newPose.pose.orientation.z = orientation.z();
	newPose.pose.orientation.w = orientation.w();

	path_->poses.push_back(newPose);
}

nav_msgs::Path::Ptr Trajectory::getPath(bool updateStamp, const string& frameId) {
	path_->header.frame_id = frameId;

	foreach(geometry_msgs::PoseStamped& pose, path_->poses) {
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = frameId;
	}

	return path_;
}

void Trajectory::setWeight(double weight) {
	if (weight < 0 || weight > 1)
		throw new std::invalid_argument("weight");

	weight_ = weight;
}

void Trajectory::setFactors(double f1, double f2)
{
	f1_ = f1;
	f2_ = f2;
}

double Trajectory::getF1()const
{
	return f1_;
}

double Trajectory::getF2()const
{
	return f2_;
}

void Trajectory::rotate(double angle) {
	tf::Stamped<tf::Transform> rotationTf;
	rotationTf.setOrigin(tf::Vector3(0, 0, 0));
	rotationTf.setRotation(tf::createQuaternionFromYaw(angle));

	foreach(geometry_msgs::PoseStamped& pose, path_->poses) {
		tf::Stamped<tf::Transform> tf;
		tf::poseStampedMsgToTF(pose, tf);
		tf.setData(rotationTf * tf);
		tf::poseStampedTFToMsg(tf, pose);
	}
}

/**
 * Returns true if all poses on path are at the same x-y coordinates
 * @return
 */
bool Trajectory::isInPlace() const {

	if (overrideInPlace_)
		return overrideInPlaceValue_;

	tf::Vector3 startPose;

	if (path_->poses.size() == 0)
		return true;

	tf::pointMsgToTF(path_->poses[0].pose.position, startPose);

	foreach(geometry_msgs::PoseStamped& pose, path_->poses) {
		tf::Vector3 pathPose;

		tf::pointMsgToTF(pose.pose.position, pathPose);

		if (pathPose.distance(startPose) > 1e-5)
			return false;
	}

	return true;
}

/**
 * Returns true if the x coordinate of the second pose in path is
 * greater than the x coordinate of the first pose
 * @return
 */
bool Trajectory::isForward() const {
	if (path_->poses.size() <= 1)
		return false;

	return path_->poses[1].pose.position.x - path_->poses[0].pose.position.x > 0;
}

void Trajectory::concatenate(const Trajectory& trajectory) {

	tf::Vector3 pose(0, 0, 0);
	tf::Quaternion orientation(0, 0, 0, 1);

	if (this->path_->poses.size() > 0) {
		tf::pointMsgToTF(this->path_->poses[this->path_->poses.size() - 1].pose.position, pose);
		tf::quaternionMsgToTF(this->path_->poses[this->path_->poses.size() - 1].pose.orientation, orientation);
	}

	tf::Transform trajectoryFrame(orientation, pose);

	const nav_msgs::Path::Ptr& newPath = trajectory.getPath();

	for (int i = 0; i < newPath->poses.size(); ++i) {
		tf::pointMsgToTF(newPath->poses[i].pose.position, pose);
		tf::quaternionMsgToTF(newPath->poses[i].pose.orientation, orientation);

		tf::Transform poseTf(orientation, pose);

		tf::Transform newPose = trajectoryFrame * poseTf;

		addPosition(newPose.getOrigin(), newPose.getRotation());
	}
}
