/*
 * test.cpp
 *
 *  Created on: May 28, 2014
 *      Author: dan
 */
#include <ros/ros.h>
#include <tf_geometry/tf_geometry.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace tf_geometry;
typedef cryph::AffPoint AffVector;
ostream& operator<<(ostream& o, const tf::Vector3& v){
	o<<"("<<v.x()<<","<<v.y()<<","<<v.z()<<")";
	return o;
}

int main(int a, char** aa){
	ros::init(a, aa, "tf_test");

	AffVector v( 0.123, 321, 0.442 );
	Affine m;

//	m *= m.translation(AffVector(19,29,19.43));
//	m *= m.zRotationRadians(25*d2r);
	m = Pose::AffineIdentity() % Pose::AffineRotate(0,0,25*d2r) % Pose::AffineTranslate(19,29,19.43);

	cout<<" v "<<v <<endl;
	v = m * v;

	cout<<" v "<<v <<endl;

	Position p( 0.123, 321, 0.442 );
	Position trans = Position(19,29,19.43);
	Orientation rot = Orientation(25*d2r,0,0);
	Pose pp1(trans);
	Pose pp2(rot);

	cout<<" p "<< p.to_tfVector3() <<endl;
	p = p % pp2 % pp1;
	cout<<" p "<< p.to_msg_Point() <<endl;

	return 0;
}


