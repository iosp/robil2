/*
 * EKF_properties.h
 *
 *  Created on: Apr 24, 2014
 *      Author: lar5
 */

#ifndef EKF_PROPERTIES_H_
#define EKF_PROPERTIES_H_
#include <opencv2/opencv.hpp>
using namespace cv;

class ekf_props{
public:
	int s,m;
    double T;
    Mat F, Q, H, R, xk,P,xk1,P1,K,z;
	void __init__props()
	{
		s = 6;
		m = 5;
		T = 0.1;
		F  = (Mat_<double>(s,s) << 1, 0, 0, 0, 0, 0,//T*T/2,
								   0, 1, 0, 0, 0, 0,//T*T/2,
								   0, 0, 1, 0, T, 0,
								   0, 0, 0, 1, 0, T,
								   0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0);

		xk = Mat::zeros(s,1, CV_64F);
		xk1 = xk;
		z = Mat::zeros(m,1, CV_64F);
		////     x
		////     y
		////     theta
		////     v
		////     w
		////     a

		Q = Mat::eye(s, s, CV_64F)*0.002;

		H = (Mat_<double>(m,s) << 1, 0, 0, 0, 0, 0,
								  0, 1, 0, 0, 0, 0,
								  0, 0, 1, 0, 0, 0,
								  0, 0, 0, 0, 1, 0,
								  0, 0, 0, 0, 0, 1);

		R = (Mat_<double>(m,m) << 6.25*0.00001, 0, 0, 0, 0,
									  0, 6.25*0.00001, 0, 0, 0,
									  0, 0,  0.8, 0, 0,
									  0, 0,    0, 0.000252982*100, 0,
									  0, 0,    0, 0, 0.000465329*100);
		P = H.t()*R*H;
		P1 = F*P*F.t() + Q;

	}
	void modify_F(double T,double theta)
	{
		this->F  = (Mat_<double>(s,s) << 1, 0, 0, T*cos(theta), 0, cos(theta)*T*T/2,
										   0, 1, 0, T*sin(theta), 0, sin(theta)*T*T/2,
										   0, 0, 1, 0, T, 0,
										   0, 0, 0, 1, 0, T,
										   0, 0, 0, 0, 0, 0,
										   0, 0, 0, 0, 0, 0);
	}
};

#endif /* EKF_PROPERTIES_H_ */
