/*
 * xml_args.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_TAO_PARSER_XML_ARGS_H_
#define SRC_TAO_PARSER_XML_ARGS_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace xml_args{
using namespace core;

bool contains_xml_argument( TagPtr tag, const string& name, const Context& context );
string get_xml_argument( TagPtr tag, const string& name, const Context& context );
string get_xml_argument( TagPtr tag, const string& name , string default_value, const Context& context );
map<string,string> get_xml_arguments( TagPtr tag, Requerments requered, const Context& context );
list<string> get_xml_arguments_names( TagPtr tag, Requerments requered, const Context& context );


}}}}}

#endif /* SRC_TAO_PARSER_XML_ARGS_H_ */
