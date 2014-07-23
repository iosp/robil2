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

















