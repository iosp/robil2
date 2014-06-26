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
		s = 11;
		m = 10;
		dt = 0.1;
		tk = t;
		xk = Mat::zeros(s,1, CV_64F);
		modify_F(0,0);
		xk1 = xk;
		//0//     x
		//1//     y
		//2//     z
		//3//     v
		//4//     a
		//5//     R
		//6//     P
		//7//     Y
		//8//     dR
		//9//     dP
		//10/     dY

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


		Q = Mat::eye(s, s, CV_64F)*0.002;

		H = (Mat_<double>(m,s) <<  1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
					    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
					    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
					    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
					    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
					    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
					    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
					    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
		    );

		R = (Mat_<double>(m,m) << Vgps,  0, 0, 0, 0, 0, 0, 0, 0, 0,
								  0,  Vgps, 0, 0, 0, 0, 0, 0, 0, 0,
								  0, 0, Vvel, 0, 0, 0, 0, 0, 0, 0,
								  0, 0, 0,  Vacc, 0, 0, 0, 0, 0, 0, 
								  0, 0, 0, 0, Vori, 0, 0, 0, 0, 0,
								  0, 0, 0, 0, 0, Vori, 0, 0, 0, 0,
								  0, 0, 0, 0, 0, 0, Vori, 0, 0, 0,
								  0, 0, 0, 0, 0, 0, 0, Vgyro, 0, 0,
								  0, 0, 0, 0, 0, 0, 0, 0, Vgyro, 0,
								  0, 0, 0, 0, 0, 0, 0, 0, 0, Vgyro
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
		if (dt < 0.005)
		{
			dt = 0.005;
			std::cout << "dt= " << dt << std::endl;
		}
		tk = tk1;
		modify_F(xk.at<double>(7,0),xk.at<double>(6,0));
	}
	void modify_F(double yaw,double pitch)
	{
		F  = (Mat_<double>(s,s) << 1, 0, 0, dt*cos(yaw)*cos(pitch),    dt*dt*cos(yaw)*cos(pitch)/2, 0, 0, 0, 0, 0, 0,
								   0, 1, 0, dt*sin(yaw)*cos(pitch),    dt*dt*sin(yaw)*cos(pitch)/2, 0, 0, 0, 0, 0, 0,
								   0, 0, 1, dt*sin(pitch),    dt*dt/2*sin(pitch), 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 1,   dt, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0,    0, 1, 0, 0, 1, 0, 0,
								   0, 0, 0, 0,    0, 0, 1, 0, 0, 1, 0,
								   0, 0, 0, 0,    0, 0, 0, 1, 0, 0, 1,
								   0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0
								   );
	}
public:
	static const double Vacc = 0.000465329;
	static const double Vgps = 6.25;
	static const double Vvel = 0.05;
	static const double Vgyro = 0.000252982;
	static const double Vori = 5/180*3.14159;
};

#endif /* EKF_PROPERTIES_H_ */
