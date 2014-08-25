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

namespace InverseKinematics{

float get_J(float);
float get_H(float);
float get_pitch(float,float);
double SupporterInv(double q3);
double LoaderInv(double q3, double q5);

}

#endif /* INVERSEKINEMATICS_H_ */
