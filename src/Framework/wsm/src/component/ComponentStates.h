/*
 * ComponentStates.h
 *
 *  Created on: Feb 10, 2014
 *      Author: dan
 */

#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_

#include "ComponentMain.h"
#include <gazebo_msgs/GetModelState.h>
#include "helpermath.h"
#include "aux_functions.h"

geometry_msgs::Twist Translate(gazebo_msgs::GetModelState model_state);
void runComponent(int argc, char** argv, ComponentMain& component);
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}


#endif /* COMPONENTSTATES_H_ */
