/*
 * PLPCompiler.h
 *
 *  Created on: May 7, 2014
 *      Author: dan
 */

#ifndef PLPCOMPILER_H_
#define PLPCOMPILER_H_

#include "PLPDataStruct.h"
#include "MonitoringScript.h"
using namespace std;

class PLPCompiler {
public:
	PLPCompiler();
	virtual ~PLPCompiler();

	vector<MonitorningScript> compile(const PLP& plp, int& error_code);
	vector<MonitorningScript> compile_testing(const PLP& plp, int& error_code);

};

#endif /* PLPCOMPILER_H_ */
