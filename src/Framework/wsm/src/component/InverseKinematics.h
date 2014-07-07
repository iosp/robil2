/*
 * InverseKinematics.h
 *
 *  Created on: Jul 7, 2014
 *      Author: lab116-2
 */

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <cmath>

namespace InverseKinematics{

	double SupporterInv(double q3);
	double LoaderInv(double q3, double q5);

}

#endif /* INVERSEKINEMATICS_H_ */
