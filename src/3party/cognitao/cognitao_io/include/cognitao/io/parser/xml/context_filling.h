/*
 * context_filling.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_CONTEXT_FILLING_H_
#define SRC_CONTEXT_FILLING_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace context_filling{
using namespace core;

void add_folder_to_path( Context& context, const string& folder );
void add_argument( Context& context, const string& arg_name, const string& arg_value );

}}}}}


#endif /* SRC_CONTEXT_FILLING_H_ */
