/*
 * XMLParser.cpp
 *
 *  Created on: Nov 21, 2015
 *      Author: dan
 */

#include <cognitao/io/parser/xml/XMLParser.h>

#include <map>
#include <string>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{

XMLParser::XMLParser() {

}

XMLParser::~XMLParser() {
}

cognitao::io::parser::core::Node XMLParser::adapter( const xml::core::TagPair& tag, const xml::core::Context& context )const
{
	xml::core::TagPtr start = tag.open;
	xml::core::TagPtr end = tag.closed+1;
	parser::core::Node node;
	parser::core::Node::Location loc;

	loc.source = tag.open->source;
	loc.open_line = tag.open->open_line;
	loc.open_offset_in_line = tag.open->open_offset_in_line;
	loc.open_offset_in_file = tag.open->open_offset_in_file;
	loc.close_line = tag.closed->close_line;
	loc.close_offset_in_line = tag.closed->close_offset_in_line;
	loc.close_offset_in_file = tag.closed->close_offset_in_file;

	node.set_name( start->name );
	node.set_location( loc );

	typedef std::map<std::string,std::string> Arguments;
	Arguments arguments = xml::xml_args::get_xml_arguments(start, xml::core::Required, context);
	if( arguments.empty() == false )
	{
		BOOST_FOREACH( Arguments::value_type& a, arguments )
		{
			node.add_arg( a.first, a.second );
		}
	}

	if( tag.has_body() )
	{
		node.set_data( xml::tokens::to_string(start+1, end-1) );

		for( xml::core::TagPtr cursor = start+1; cursor!=tag.closed; cursor++ )
		{
			xml::core::TagPair sub_tag;
			if( cursor->name == xml::core::Tag::DATA_NAME() ) continue;
			if( cursor->name == xml::core::Tag::COMENT_NAME() ) continue;
			if( cursor->name == xml::core::Tag::VERSION_NAME() ) continue;

			if( xml::tokens::find( cursor->name, cursor, end, sub_tag, xml::core::Required, false) )
			{
				parser::core::Node sub_node = adapter( sub_tag , context );
				node.add_node( sub_node );

				cursor = sub_tag.closed;
			}
		}
	}

	return node;
}


cognitao::io::parser::core::MachinesCollection XMLParser::adapter( xml::core::MachinesTable& machines_tags, const std::string& start, const xml::core::Context& context )const
{
	cognitao::io::parser::core::MachinesCollection machines;
	machines.start_machine = start;
	BOOST_FOREACH( xml::core::MachinesTable::value_type& m, machines_tags )
	{
		parser::core::Node node = adapter( xml::core::TagPair( m.second.begin(), m.second.end()-1 ) , context );
		machines.machines[ m.first ] = ( node );
	}
	return machines;
}


cognitao::io::parser::core::MachinesCollection XMLParser::parse( std::istream& stream, const std::string& source )
{
	xml::core::Context context;
	core::MachinesTable machines;
	std::string start_machine;
	xml::iostream::parse_file( stream, source, context, machines, start_machine );
	return adapter(machines, start_machine, context);
}

cognitao::io::parser::core::MachinesCollection XMLParser::parse( const std::string& file_name)
{
	xml::core::Context context;
	core::MachinesTable machines;
	std::string start_machine;
	xml::iostream::parse_file( file_name, context, machines, start_machine );
	return adapter(machines, start_machine, context);
}

}}}}
