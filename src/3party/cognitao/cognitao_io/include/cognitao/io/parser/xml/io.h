/*
 * tao_parser_io.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef TAO_PARSER_IO_H_
#define TAO_PARSER_IO_H_


#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace iostream{

using namespace core;

vector<string> split(const string& source, const string& del="\n");
std::string exec(const string cmd);

bool read_file( string file_name, string& xml );
bool parse_file( string file_name, property_tree::ptree& pt );
bool parse_file( string file_name, const Context& global_context, MachinesTable& machines, std::string& start_machine );
bool parse_file( istream& stream, const string& source, const Context& global_context, MachinesTable& machines, std::string& start_machine );

bool parse_file_for_subs( string file_name, Context& global_context );
bool parse_file_for_subs( istream& stream, const string& source, Context& global_context );

bool include_file( string file_name, Context& global_context );
bool include_file( istream& stream, const string& source, Context& global_context );


}}}}}


#endif /* TAO_PARSER_IO_H_ */
