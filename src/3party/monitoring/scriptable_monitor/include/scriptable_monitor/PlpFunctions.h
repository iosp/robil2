/*
 * SomeInternalFunctionExample.h
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#ifndef PLPFUNCTIONS_H_
#define PLPFUNCTIONS_H_
#include <scriptable_monitor/CustomFunctionsHeader.h>

FUNCTION(get_global_var_float)
FUNCTION(set_global_var_float)

FUNCTION(get_global_var_str)
FUNCTION(set_global_var_str)

FUNCTION(get_global_var_int)
FUNCTION(set_global_var_int)


FUNCTION(Now)
FUNCTION(get_module_status)

FUNCTION(remove_script)

#undef FUNCTION
#endif /* PLPFUNCTIONS_H_ */
