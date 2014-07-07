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

namespace InverseKinematics{

	double SupporterInv(double q3){
		return 2.6129*sin(acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*sin(phi + asin((1.7323*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + asin((1.0486*sin(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*(((1.7787*sin(q3 + q30)*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)) + (1.7967*sin(q3 + q30)*(1.0372*cos(q3 + q30) - 0.64094))/(pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2))*pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))/pow(((3.0009*(pow((1.0372*cos(q3 + q30) - 0.64094),2) - 1.0))/(2.0536*cos(q3 + q30) + 2.0584) + 1.0),(1.0/2)) + ((1.0486*cos(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2)) + (1.0767*pow(sin(q3 + q30),2))/pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)))/pow((1.0 - (1.0996*pow(sin(q3 + q30),2))/(2.0536*cos(q3 + q30) + 2.0584)),(1.0/2))) - 2.6129*cos(acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*cos(phi + asin((1.7323*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + asin((1.0486*sin(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*(((1.7787*sin(q3 + q30)*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)) + (1.7967*sin(q3 + q30)*(1.0372*cos(q3 + q30) - 0.64094))/(pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2))*pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))/pow(((3.0009*(pow((1.0372*cos(q3 + q30) - 0.64094),2) - 1.0))/(2.0536*cos(q3 + q30) + 2.0584) + 1.0),(1.0/2)) + ((1.0486*cos(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2)) + (1.0767*pow(sin(q3 + q30),2))/pow((2.0536*cos(q3 + q30) + 2.0584),(3.0/2)))/pow((1.0 - (1.0996*pow(sin(q3 + q30),2))/(2.0536*cos(q3 + q30) + 2.0584)),(1.0/2))) - (0.59273*cos(acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*sin(q3 + q30))/pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)) + (2.71*cos(acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*cos(phi + asin((1.7323*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + asin((1.0486*sin(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*sin(q3 + q30))/pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)) - (2.71*sin(acos(0.64094 - 1.0372*cos(q3 + q30)) - 0.2874)*sin(phi + asin((1.7323*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + asin((1.0486*sin(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))))*sin(q3 + q30))/pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2));
	}

	double LoaderInv(double q3, double desired_pitch){
		double q1 = acos(0.64094 - 1.0372*cos(q3 + q30));
		double q2 = asin((1.7323*pow((1.0 - 1.0*pow((1.0372*cos(q3 + q30) - 0.64094),2)),(1.0/2)))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2))) + asin((1.0486*sin(q3 + q30))/pow((2.0536*cos(q3 + q30) + 2.0584),(1.0/2)));
		double q5 = desired_pitch - ( -q1 + alpha - M_PI - q2 - phi - q50);
		q5 = fmod(q5 + M_PI ,2*M_PI) - M_PI;
		double pitch = -(q1 + alpha) + M_PI + q2 + phi +q50 +q5;
		pitch = fmod(pitch + M_PI ,2*M_PI) - M_PI;


		ROS_INFO("pitch = %f",pitch);
		ROS_INFO("q1 = %f",q1);
		ROS_INFO("q2 = %f",q2);
		ROS_INFO("q5 = %f",q5);
		ROS_INFO("desired_pitch = %f",desired_pitch);


		return 0;
	}

}
