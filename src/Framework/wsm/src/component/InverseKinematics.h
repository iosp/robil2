/*
 * InverseKinematics.h
 *
 *  Created on: Jul 7, 2014
 *      Author: lab116-2
 */

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <stdlib.h>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Float32.h>
#include "tf/tf.h"

namespace InverseKinematics{

typedef struct RotMatrix { double R[4][4] ; } RotMatrix;

float get_J(float);
float get_H(float);
float get_pitch(float,float);
double SupporterInv(double q3);
double LoaderInv(double q3, double q5);
RotMatrix Matrix_mul(RotMatrix lhs , RotMatrix rhs );
RotMatrix getRotMatrix (geometry_msgs::PoseWithCovarianceStamped *q);

}

#endif /* INVERSEKINEMATICS_H_ */
