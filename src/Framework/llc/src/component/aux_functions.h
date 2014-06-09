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

double norm(std_msgs::Float64 *vec , int size){
	double Norm = 0 ;
	for (int i=0 ; i < size ; i++){
		Norm += vec[i].data * vec[i].data ;
	}
	return Norm;
}

double dot_prod(std_msgs::Float64 *vec_a ,std_msgs::Float64 *vec_b , int size ){
	double prod = 0;
		for(int i =0 ; i < size ; i++){
			prod += vec_a[i].data * vec_b[i].data;
		}
	return prod;
}
