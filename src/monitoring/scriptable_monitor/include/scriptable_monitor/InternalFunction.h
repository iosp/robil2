/*
 * InternalFunction.h
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#ifndef INTERNALFUNCTION_H_
#define INTERNALFUNCTION_H_

#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include <scriptable_monitor/Parameters.h>

using namespace std;

#define DEFINE_INTERNAL_FUNCTION(FunctionClassName) \
class FunctionClassName:public InternalFunction {\
public:\
FunctionClassName() { }\
	virtual ~FunctionClassName() { }\
	virtual string functionName(){ return #FunctionClassName; };\
\
protected:\
	virtual void process(Parameters& input, Parameters& output);\
};\
REGISTER_INTERNAL_FUNCTION(FunctionClassName)\
void NAME::process(Parameters& input, Parameters& output)

#define REGISTER_INTERNAL_FUNCTION(FunctionClassName) \
static InternalFunctionRegistrar FunctionClassName##registrar(new FunctionClassName());


/**
 * Interface for a custom internal function
 */
class InternalFunction {
public:

	InternalFunction() { }
	virtual ~InternalFunction() { }

	Parameters invoke(Parameters& input) {
		Parameters output;
		process(input, output);
		return output;
	}

	virtual string functionName() = 0;

protected:
	virtual void process(Parameters& input, Parameters& output) = 0;
};


/**
 * Manages internal functions lists with register/resolve methods
 */
class InternalFunctionsManager {
public:
	static void registerFunction(InternalFunction* function) {
		_functions[function->functionName()] = function;
	}

	static InternalFunction* resolve(string functionName) {
		if (_functions.count(functionName) == 0)
			return NULL;

		return _functions[functionName];
	}

	static vector<string> getFunctionNames() {

		pair<string, InternalFunction*> func;
		vector<string> names;

		BOOST_FOREACH(func, _functions) {
			names.push_back(func.first);
		}

		return names;
	}
private:
	static map<string, InternalFunction*> _functions;
};


class InternalFunctionRegistrar {
public:
	InternalFunctionRegistrar(InternalFunction* function) {
		InternalFunctionsManager::registerFunction(function);
	}
};



#endif /* INTERNALFUNCTION_H_ */
