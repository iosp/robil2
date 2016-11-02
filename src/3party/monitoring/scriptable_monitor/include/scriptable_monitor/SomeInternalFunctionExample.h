/*
 * SomeInternalFunctionExample.h
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#ifndef SOMEINTERNALFUNCTIONEXAMPLE_H_
#define SOMEINTERNALFUNCTIONEXAMPLE_H_


#include <scriptable_monitor/InternalFunction.h>

class SomeFunction : public InternalFunction {
public:

	virtual string functionName();

protected:
	virtual void process(Parameters& input, Parameters& output);
};


#endif /* SOMEINTERNALFUNCTIONEXAMPLE_H_ */
