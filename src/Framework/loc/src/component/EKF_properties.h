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
	bool gas_pedal_state;
	int s,m;
    double dt,tk;
    double Eacc, _dIMU,_dGPS;;
    Mat F, Q, H, R, xk,P,xk1,P1,K,z,u,B;
	void __init__props(double t)
	{
		Eacc = 0.137;
		s = 13;
		m = 13;
		dt = 0.05;
		tk = t;
		xk = Mat::zeros(s,1, CV_64F);
		xk1 = xk;
		//0//     x
		//1//     y
		//2//     z
		//3//     vx
		//4//     vy
		//5//     ax
		//6//     ay
		//7//     R
		//8//     P
		//9//     Y
		//10//     dR
		//11//     dP
		//12/     dY

		z = Mat::zeros(m,1, CV_64F);
		////0     x
		////1     y
		////2     v
		////3     a
		////4     R
		////5     P
		////6     Y
		////7     dR
		////8     dP
		////9     dY
		u = Mat::zeros(2,1, CV_64F);
		B = Mat::zeros(s,2,CV_64F);
		F = Mat::eye(s,s,CV_64F);
		modify_F(0,0);
		////0     Throttle input
		////1     Steering input
		Q = Mat::eye(s, s, CV_64F)*0.002;

		H = Mat::eye(s, s, CV_64F);
		Mat arr = (Mat_<double>(1,m) <<  Vgps,Vgps,Vz,Vvel,Vvel,Vacc,Vacc,Vori,Vori,Vori,Vgyro,Vgyro,Vgyro);
		R = Mat::diag(arr);
		P = H.t()*R*H;
		P1 = F*P*F.t() + Q;
	}
	void modify_Q(double val)
	{
		Q = Mat::eye(s, s, CV_64F)*val;
	}
	void setdt(double tk1)
	{
		dt = tk1 - tk;
		if (dt < 0.005)
		{
			dt = 0.005;
			std::cout << "LOC says: dt= " << dt << std::endl;
		}
		tk = tk1;
		modify_F(xk.at<double>(9,0),xk.at<double>(8,0));
		modify_B(xk.at<double>(9,0),xk.at<double>(8,0));
	}
	void modify_F(double yaw,double pitch)
	{
	  F.at<double>(0,0) = 1;F.at<double>(0,3) = dt;F.at<double>(0,5) = dt*dt/2;
	  F.at<double>(1,1) = 1;F.at<double>(1,4) = dt;F.at<double>(1,6) = dt*dt/2;
	  F.at<double>(3,3) = 0;
	  F.at<double>(4,4) = 0;
	  F.at<double>(9,9) = 1;
	  F.at<double>(9,12) = dt;
	  F.at<double>(12,12) = 0;
	}
	void modify_B(double yaw,double pitch)
	{
		B.at<double>(3,0) = cos(yaw);
		B.at<double>(4,0) = sin(yaw);
		//B.at<double>(9,1) = dt;
		B.at<double>(12,1) = 1;
	}
public:
	static const double Vacc = 0.0465329;
	static const double Vgps = 30.25;
	static const double Vz = 0.001;
	static const double Vvel = 0.05;
	static const double Vgyro = 0.000252982;
	static const double Vori = 0.087266389;
};

#endif /* EKF_PROPERTIES_H_ */
