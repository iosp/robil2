#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

double power_x(double x,int powX)
{
	double val = 1;
	for (int i=0;i<powX;i++)
		val *= x;
	return val;
}

double calculate_m(vector<Point> Ps, int powX, int powY, int size)
{
	double m = 0;
	for (int i=0;i<size;i++)
		m += power_x(Ps[i].y,powX) * power_x(Ps[i].x,powY);
	return m;	
}

void polyfit(double *a, vector<Point> Ps, int size)
{
	double m1 = calculate_m(Ps,1,0,size);
	double m2 = calculate_m(Ps,2,0,size);
	double m3 = calculate_m(Ps,3,0,size);
	double m4 = calculate_m(Ps,4,0,size);
	double my = calculate_m(Ps,0,1,size);
	double mxy = calculate_m(Ps,1,1,size);
	double mx2y = calculate_m(Ps,2,1,size);
	double al = m4*m1*m1 - 2*m1*m2*m3 + m2*m2*m2 - m4*size*m2 + size*m3*m3;
	double b1 = m1*m4 - m2*m3;
	double b2 = m2*m2 - m1*m3;
	double b3 = size*m3 - m1*m2;
	a[0] = ((m3*m3 - m2*m4)*my + mxy*b1 + mx2y*b2)/al;
	a[1] = (b1*my + (m2*m2 - size*m4)*mxy + b3*mx2y)/al;
	a[2] = (b2*my + b3*mxy + (m1*m1 - size*m2)*mx2y)/al;
}
