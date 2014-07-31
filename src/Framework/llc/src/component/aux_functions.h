#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

double glSpeedloc(double speedx,double speedy, double yaw)
{
	double theta = atan2(speedy,speedx);
	double eps = 3.14 / 4;
	if (yaw > theta - eps && theta + eps > yaw)
		return sqrt(speedx*speedx + speedy*speedy);
	return -sqrt(speedx*speedx + speedy*speedy);
}

double norm(std_msgs::Float64 *vec , int size)
{
	double Norm = 0 ;
	for (int i=0 ; i < size ; i++){
		Norm += vec[i].data * vec[i].data ;
	}
	return Norm;
}

double dot_prod(std_msgs::Float64 *vec_a ,std_msgs::Float64 *vec_b , int size )
{
	double prod = 0;
		for(int i =0 ; i < size ; i++){
			prod += vec_a[i].data * vec_b[i].data;
		}
	return prod;
}
/*
double MedianOfFive(double* arr)
{
	double a = arr[0];
	double b = arr[1];
	double c = arr[2];
	double d = arr[3];
	double e = arr[4];
    return b < a ? d < c ? b < d ? a < e ? a < d ? e < d ? e : d
                                                 : c < a ? c : a
                                         : e < d ? a < d ? a : d
                                                 : c < e ? c : e
                                 : c < e ? b < c ? a < c ? a : c
                                                 : e < b ? e : b
                                         : b < e ? a < e ? a : e
                                                 : c < b ? c : b
                         : b < c ? a < e ? a < c ? e < c ? e : c
                                                 : d < a ? d : a
                                         : e < c ? a < c ? a : c
                                                 : d < e ? d : e
                                 : d < e ? b < d ? a < d ? a : d
                                                 : e < b ? e : b
                                         : b < e ? a < e ? a : e
                                                 : d < b ? d : b
                 : d < c ? a < d ? b < e ? b < d ? e < d ? e : d
                                                 : c < b ? c : b
                                         : e < d ? b < d ? b : d
                                                 : c < e ? c : e
                                 : c < e ? a < c ? b < c ? b : c
                                                 : e < a ? e : a
                                         : a < e ? b < e ? b : e
                                                 : c < a ? c : a
                         : a < c ? b < e ? b < c ? e < c ? e : c
                                                 : d < b ? d : b
                                         : e < c ? b < c ? b : c
                                                 : d < e ? d : e
                                 : d < e ? a < d ? b < d ? b : d
                                                 : e < a ? e : a
                                         : a < e ? b < e ? b : e
                                                 : d < a ? d : a;
}
*/
void Push_elm(double* arr , int size , double elm)
{
		for(int i = 0 ; i < (size - 1) ; i++ )
			arr[i+1] = arr[i];
		arr[0] = elm ;
	return;
}

//   1D MEDIAN FILTER implementation
//     signal - input signal
//     result - output signal
//     N      - length of the signal
double _medianfilter(const double* signal, int N)
{
	double data[N] ;
	for (int i = 0 ; i < N ; i++)
		data[i] = signal[i];
	int i,j;
	double temp ;

	for (i=0; i<N-1; i++) {
		for (j=0; j<N-1-i; j++) {
			if (data[j] > data[j+1])
			{
				temp=data[j];
				data[j]=data[j+1];
				data[j+1]=temp;
			}
		}
	}
	return data[N/2 + 1];
}

double SGfilter(const double* signal)
{
	/*
	double f_coeff[21] = {0.0447204968944098 , -0.0248447204968945 , -0.0500163452108533 , -0.0431513566525009 , -0.0151529719439264 , 0.0245293540757263 , 0.0678999288502586, 0.108416822106417 ,
						  0.140991865853893  , 0.161990654385324 , 0.169232544276292 , 0.161990654385324 , 0.140991865853893 , 0.108416822106417 , 0.0678999288502586 , 0.0245293540757263 , -0.0151529719439263,
						  -0.0431513566525009 , -0.0500163452108534 , -0.0248447204968946 , 0.0447204968944096 } ;
	*/

	double f_coeff[21] = {-0.0270531400966188,
0.0541062801932374,
0.0210017798118489,
-0.0329519450800914,
-0.0595504105532375,
-0.0429667519181588,
0.0113698568672320,
0.0861847713913943,
0.160057432584018,
0.213409910112024,
0.232784433376707,
0.213409910112023,
0.160057432584017,
0.0861847713913938,
0.0113698568672317,
-0.0429667519181586,
-0.0595504105532372,
-0.0329519450800911,
0.0210017798118489,
0.0541062801932372,
-0.0270531400966180} ;

	double output;

		for(int i = 0 ; i < 21 ; i++){
			output += signal[i] * f_coeff[i];
		}

		return output ; /* return y[0] */
}















