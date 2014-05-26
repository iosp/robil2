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
    Mat F, Q, H, R, xk,P,xk1,P1,K,z;
	void __init__props(double t)
	{
		Eacc = 0.137;
		s = 6;
		m = 5;
		dt = 0.1;
		tk = t;
		xk = Mat::zeros(s,1, CV_64F);
		modify_F(0);
		xk1 = xk;
		////     x
		////     y
		////     v
		////     a
		////     theta
		////     dtheta

		z = Mat::zeros(m,1, CV_64F);
		////     x
		////     y
		////     a
		////     theta
		////     dtheta


		Q = Mat::eye(s, s, CV_64F)*0.002;

		H = (Mat_<double>(m,s) << 1, 0, 0, 0, 0, 0,
								  0, 1, 0, 0, 0, 0,
								  0, 0, 0, 1, 0, 0,
								  0, 0, 0, 0, 1, 0,
								  0, 0, 0, 0, 0, 1
								  );

		R = (Mat_<double>(m,m) << Vgps, 0, 0, 0, 0,
								  0, Vgps, 0, 0, 0,
								  0, 0, Vacc, 0, 0,
								  0, 0, 0, Vgyro, 0,
								  0, 0, 0, 0, Vgyro
								  );

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
		if (dt < 0.05)
		{
			dt = 0.05;
			std::cout << "dt= " << dt << std::endl;
		}
		tk = tk1;
		modify_F(xk.at<double>(4,0));
	}
	void modify_F(double theta)
	{
		F  = (Mat_<double>(s,s) << 1, 0, dt*cos(theta),    dt*dt*cos(theta)/2, 0, 0,
								   0, 1, dt*sin(theta),    dt*dt*sin(theta)/2, 0, 0,
								   0, 0, 1,   dt, 0, 0,
								   0, 0, 0,    0, 0, 0,
								   0, 0, 0,    0, 1, 0,
								   0, 0, 0,    0, 0, 1
								   );
	}
public:
	static const double Vacc = 0.000465329;
	static const double Vgps = 6.25;
	static const double Vgyro = 0.000252982;
};

#endif /* EKF_PROPERTIES_H_ */
