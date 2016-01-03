/*
 * SomeInternalFunctionExample.cpp
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/SomeInternalFunctionExample.h>

REGISTER_INTERNAL_FUNCTION(SomeFunction)

string SomeFunction::functionName()
{
	return "SomeFunction";
}

void SomeFunction::process(Parameters& input, Parameters& output)
{
	size_t size = input.size();
	double sum = 0;

	for(size_t i = 0; i < size; i++) {
		double value = input.get<double>(i);
		sum += value;
	}

	double average = sum / size;

	output.set<double>(average);
}
