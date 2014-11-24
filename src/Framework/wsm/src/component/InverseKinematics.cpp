/*
 * InverseKinematics.cpp
 *
 *  Created on: Jul 7, 2014
 *      Author: lab116-2
 */

#include "InverseKinematics.h"
#include <ros/ros.h>

#define q30 -1.55321
#define q50  -1 //-2.9078
#define phi (M_PI / 6.0)
#define alpha 0.2874
#define ksi 0.0


#define WRAP_POSNEG_PI(x) atan2(sin(x), cos(x))
#define LIMIT(value, minim, maxim) std::max<double>(std::min<double>(value, maxim), minim)
#define safe_asin(x) asin(LIMIT(x, -1, 1))
#define safe_acos(x) acos(LIMIT(x, -1, 1))
#define safe_pow(x, y) ((fabs(y - 0.5) < 0.001) ? sqrt(fabs(x)) : pow(x, y))

namespace InverseKinematics{

float get_J(float q3) {

	float J = 2.6129546*sin(asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + 0.5142197)*sin(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978)*(((1.0767244*pow(sin(q3 - 1.03525),2))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(3.0/2)) + (1.0485848*cos(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2)))/pow((1.0 - (1.09953*pow(sin(q3 - 1.03525),2))/(2.0536716*cos(q3 - 1.03525) + 2.0584777)),(1.0/2)) + ((1.7797528*sin(q3 - 1.03525)*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(3.0/2)) + (1.7966802*sin(q3 - 1.03525)*(1.0366021*cos(q3 - 1.03525) - 0.64218847))/(pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2))))/pow(((3.0041206*(pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2) - 1.0))/(2.0536716*cos(q3 - 1.03525) + 2.0584777) + 1.0),(1.0/2))) - 2.6129546*cos(asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + 0.5142197)*cos(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978)*(((1.0767244*pow(sin(q3 - 1.03525),2))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(3.0/2)) + (1.0485848*cos(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2)))/pow((1.0 - (1.09953*pow(sin(q3 - 1.03525),2))/(2.0536716*cos(q3 - 1.03525) + 2.0584777)),(1.0/2)) + ((1.7797528*sin(q3 - 1.03525)*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(3.0/2)) + (1.7966802*sin(q3 - 1.03525)*(1.0366021*cos(q3 - 1.03525) - 0.64218847))/(pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2))))/pow(((3.0041206*(pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2) - 1.0))/(2.0536716*cos(q3 - 1.03525) + 2.0584777) + 1.0),(1.0/2))) - (0.5924372*sin(q3 - 1.03525)*cos(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978))/pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)) + (2.7085943*sin(q3 - 1.03525)*cos(asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + 0.5142197)*cos(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978))/pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)) - (2.7085943*sin(q3 - 1.03525)*sin(asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + 0.5142197)*sin(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978))/pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2));
	return J;
}

float get_H(float q3) {

	float H = 0.5715184*sin(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 0.28743978) - 2.6129546*sin(acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) + asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) + 0.22677992) + 1.1773;
	return H;
}

float get_pitch(float q3 , float loader) {

	float pitch = - 1.0*acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 1.0*asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) - 1.0*asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2)))  + 2.27 + 0.08 ;

	//float pitch = - 1.0*acos(0.64218847 - 1.0366021*cos(q3 - 1.03525)) - 1.0*asin((1.0485848*sin(q3 - 1.03525))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) - 1.0*asin((1.7332399*pow((1.0 - 1.0*pow((1.0366021*cos(q3 - 1.03525) - 0.64218847),2)),(1.0/2)))/pow((2.0536716*cos(q3 - 1.03525) + 2.0584777),(1.0/2))) - 2.27 ;
	pitch = pitch + loader;
	pitch = (pitch>=0) ? (fmod(pitch+M_PI,2*M_PI)-M_PI) : (-(fmod(-pitch+M_PI,2*M_PI)-M_PI));
	return pitch;
}

double SupporterInv(double q3){
	return 2.6129*sin(
			safe_acos(
					0.64094 - 1.0372*cos(
							q3 + q30
					)
			) - 0.2874
	)
	*
	sin(
			phi + safe_asin(
					(1.7323*safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + safe_asin((1.0486*sin(q3 + q30))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*(((1.7787*sin(q3 + q30)*safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)) + (1.7967*sin(q3 + q30)*(1.0372*cos(q3 + q30) - 0.64094))/(safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2))*safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))/safe_pow(((3.0009*(safe_pow((1.0372*cos(q3 + q30) - 0.64094),2) - 1.0))/(2.0536*cos(q3 + q30) + 2.0584) + 1.0),(1.0/2)) + ((1.0486*cos(q3 + q30))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2)) + (1.0767*safe_pow(sin(q3 + q30),2))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)))/safe_pow((1.0 - (1.0996*safe_pow(sin(q3 + q30),2))/(2.0536*cos(q3 + q30) + 2.0584)),(1.0/2))) - 2.6129*cos(safe_acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*cos(phi + safe_asin((1.7323*safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + safe_asin((1.0486*sin(q3 + q30))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*(((1.7787*sin(q3 + q30)*safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)) + (1.7967*sin(q3 + q30)*(1.0372*cos(q3 + q30) - 0.64094))/(safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2))*safe_pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))/safe_pow(((3.0009*(safe_pow((1.0372*cos(q3 + q30) - 0.64094),2) - 1.0))/(2.0536*cos(q3 + q30) + 2.0584) + 1.0),(1.0/2)) + ((1.0486*cos(q3 + q30))/sqrt((2.0536*cos(q3 + q30) + 2.0584)) + (1.0767*safe_pow(sin(q3 + q30),2))/safe_pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)))/sqrt((1.0 - (1.0996*safe_pow(sin(q3 + q30),2))/(2.0536*cos(q3 + q30) + 2.0584)))) - (0.59273*cos(safe_acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*sin(q3 + q30))/sqrt((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2))) + (2.71*cos(safe_acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*cos(phi + safe_asin((1.7323*safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/sqrt((2.0536*cos(q3 + q30) + 2.0584))) + safe_asin((1.0486*sin(q3 + q30))/sqrt((2.0536*cos(q3 + q30) + 2.0584))))*sin(q3 + q30))/safe_pow((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)) - (2.71*sin(safe_acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*sin(phi + safe_asin((1.7323*sqrt((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2))))/sqrt((2.0536*cos(q3 + q30) + 2.0584))) + safe_asin((1.0486*sin(q3 + q30))/sqrt((2.0536*cos(q3 + q30) + 2.0584))))*sin(q3 + q30))/sqrt((1.0 - 1.0*safe_pow((1.0372*cos(q3 + q30) - 0.64094),2)));
}

double LoaderInv(double q3, double desired_pitch){

	double cosq3_q30 = cos(q3 + q30);
	double sinq3_q30 = sin(q3 + q30);


	ROS_INFO("1) q3 = %f",q3);
	ROS_INFO("2) desired_pitch = %f",desired_pitch);
	ROS_INFO("3) inv: cosq3_q30: %f; sinq3_q30: %f", cosq3_q30, sinq3_q30);

	double q1 = acos(0.64094 - 1.0372*cosq3_q30);
	double q2 = asin(LIMIT(
			(1.7323*sqrt(
					1.0 - pow(1.0372*cosq3_q30 - 0.64094,2)
			)
			)
			/
			sqrt(2.0536*cosq3_q30 + 2.0584)
			, -1, 1)
	)
								+
								asin(LIMIT(1.0486*sinq3_q30
										/
										sqrt(2.0536*cosq3_q30 + 2.0584)
										, -1, 1)
								);
	double q5 = desired_pitch - ( -q1 + alpha - M_PI - q2 - phi - q50);
	q5 = WRAP_POSNEG_PI(q5);//fmod(q5 + M_PI ,2*M_PI) - M_PI;
	double pitch = -(q1 + alpha) + M_PI + q2 + phi +q50 +q5;
	pitch = WRAP_POSNEG_PI(pitch);//fmod(pitch + M_PI ,2*M_PI) - M_PI;


	ROS_INFO("4) pitch = %f",pitch);
	ROS_INFO("5) q1 = %f",q1);
	ROS_INFO("6) q2 = %f",q2);
	ROS_INFO("7) q5 = %f",q5);


	return 0;
}

RotMatrix Matrix_mul(RotMatrix lhs , RotMatrix rhs )
{
	//n * n matrices only.

	RotMatrix Mat_c ;

	for(int j = 0 ; j < 4 ; j++)
	{
		for(int i = 0 ; i < 4 ; i++)
		{
			Mat_c.R[i][j] = 0 ; 		/* init (i,j) entry */
			for(int k = 0 ; k < 4 ; k++)
			{
				Mat_c.R[i][j] += lhs.R[i][k] * rhs.R[k][j];
			}
		}
	}
/*
	ROS_INFO("R_WB");
	ROS_INFO("%g %g %g %g",lhs.R[0][0],lhs.R[0][1],lhs.R[0][2],lhs.R[0][3]);
	ROS_INFO("%g %g %g %g",lhs.R[1][0],lhs.R[1][1],lhs.R[1][2],lhs.R[1][3]);
	ROS_INFO("%g %g %g %g",lhs.R[2][0],lhs.R[2][1],lhs.R[2][2],lhs.R[2][3]);
	ROS_INFO("%g %g %g %g",lhs.R[3][0],lhs.R[3][1],lhs.R[3][2],lhs.R[3][3]);
	ROS_INFO("R_BL");
	ROS_INFO("%g %g %g %g",rhs.R[0][0],rhs.R[0][1],rhs.R[0][2],rhs.R[0][3]);
	ROS_INFO("%g %g %g %g",rhs.R[1][0],rhs.R[1][1],rhs.R[1][2],rhs.R[1][3]);
	ROS_INFO("%g %g %g %g",rhs.R[2][0],rhs.R[2][1],rhs.R[2][2],rhs.R[2][3]);
	ROS_INFO("%g %g %g %g",rhs.R[3][0],rhs.R[3][1],rhs.R[3][2],rhs.R[3][3]);
		ROS_INFO("R_WL");
		ROS_INFO("%g %g %g %g",Mat_c.R[0][0],Mat_c.R[0][1],Mat_c.R[0][2],Mat_c.R[0][3]);
		ROS_INFO("%g %g %g %g",Mat_c.R[1][0],Mat_c.R[1][1],Mat_c.R[1][2],Mat_c.R[1][3]);
		ROS_INFO("%g %g %g %g",Mat_c.R[2][0],Mat_c.R[2][1],Mat_c.R[2][2],Mat_c.R[2][3]);
		ROS_INFO("%g %g %g %g",Mat_c.R[3][0],Mat_c.R[3][1],Mat_c.R[3][2],Mat_c.R[3][3]);
*/
	return Mat_c;
}

RotMatrix getRotMatrix (geometry_msgs::PoseWithCovarianceStamped *q)
{
	RotMatrix Matrix ;

	geometry_msgs::PoseWithCovarianceStamped* r = new geometry_msgs::PoseWithCovarianceStamped(*q);

	double a = r->pose.pose.orientation.w ;
	double b = r->pose.pose.orientation.x ;
	double c = r->pose.pose.orientation.y ;
	double d = r->pose.pose.orientation.z ;

	Matrix.R[0][0] = (pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2));
	Matrix.R[1][0] = 2*b*c + 2*a*d ;
	Matrix.R[2][0] = 2*b*d - 2*a*c ;
	Matrix.R[3][0] = 0 ;

	Matrix.R[0][1] = 2*b*c - 2*a*d ;
	Matrix.R[1][1] = (pow(a,2) - pow(b,2) + pow(c,2) - pow(d,2));
	Matrix.R[2][1] = 2*c*d + 2*a*b ;
	Matrix.R[3][1] = 0 ;

	Matrix.R[0][2] = 2*b*d + 2*a*c ;
	Matrix.R[1][2] = 2*c*d - 2*a*b ;
	Matrix.R[2][2] = (pow(a,2)-pow(b,2) - pow(c,2)+pow(d,2));
	Matrix.R[3][2] = 0 ;

	Matrix.R[0][3] = r->pose.pose.position.x ;
	Matrix.R[1][3] = r->pose.pose.position.y ;
	Matrix.R[2][3] = r->pose.pose.position.z ;
	Matrix.R[3][3] = 1 ;
/*
	ROS_INFO(" X :%g",q->pose.pose.position.x);
	ROS_INFO(" Y :%g",q->pose.pose.position.y);
	ROS_INFO(" Z :%g",q->pose.pose.position.z);
*/
	delete r ;

	return Matrix ;
}

















}
