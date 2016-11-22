/*
 * Parser.h
 *
 *  Created on: Nov 21, 2015
 *      Author: dan
 */

#ifndef SRC_COGNITAO_PARSER_PARSER_H_
#define SRC_COGNITAO_PARSER_PARSER_H_

#include <cognitao/io/parser/core.h>

namespace cognitao {
namespace io {
namespace parser {

using namespace core;

class Parser {
public:
	Parser();
	virtual ~Parser();

	virtual
	core::MachinesCollection parse( std::istream& stream, const std::string& source )=0;

	virtual
	core::MachinesCollection parse( const std::string& file_name )=0;

};

}
}
}

#endif /* SRC_COGNITAO_PARSER_PARSER_H_ */
