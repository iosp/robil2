/*
 * XMLParser.h
 *
 *  Created on: Nov 21, 2015
 *      Author: dan
 */

#ifndef SRC_TAO_PARSER_XMLPARSER_H_
#define SRC_TAO_PARSER_XMLPARSER_H_

#include "../Parser.h"
#include "core.h"
#include "io.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{

class XMLParser: public Parser {
public:
	XMLParser();
	virtual ~XMLParser();

	cognitao::io::parser::core::Node adapter( const xml::core::TagPair& tag, const xml::core::Context& context )const;

	cognitao::io::parser::core::MachinesCollection adapter( xml::core::MachinesTable& machines, const std::string& start, const xml::core::Context& context )const;

	virtual
	cognitao::io::parser::core::MachinesCollection parse( std::istream& stream, const std::string& source );

	virtual
	cognitao::io::parser::core::MachinesCollection parse( const std::string& file_name );
};

}}}}

#endif /* SRC_TAO_PARSER_XMLPARSER_H_ */
