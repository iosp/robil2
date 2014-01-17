/*
 * ComponentMain.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_

#include "RosComm.h"

class ComponentMain {
	RosComm *_roscomm;
public:
	ComponentMain();
	virtual ~ComponentMain();
};

#endif /* COMPONENTMAIN_H_ */
