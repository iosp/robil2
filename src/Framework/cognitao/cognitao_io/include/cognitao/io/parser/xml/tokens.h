/*
 * xml_tokens.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_XML_TOKENS_H_
#define SRC_XML_TOKENS_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace tokens{

using namespace core;

	bool parse_to_tag_stream( istream& stream, std::string source, XMLTags& tags_stream, const ParsingParameters& params );
	bool search_first_tag(  TagPtr start, TagPtr end,  TagPtr& result  );
	bool search_closed_tag(  TagPtr start, TagPtr end,  TagPtr& result  );
	bool search_childs(  TagPtr start, TagPtr end, std::list<TagPtr>& results, bool = true );
	bool search_childs(  TagPair tag, std::list<TagPtr>& results, bool = true );
	bool find( string name,  TagPair tag, std::list<TagPtr>& results , bool search_sub_elements );
	bool find( string name,  TagPtr start, TagPtr end, std::list<TagPtr>& results , bool search_sub_elements );
	bool find_all( string name,  TagPtr start, TagPtr end, std::list<TagPtr>& results);
	bool find_all( string name,  TagPair tag, std::list<TagPtr>& results);
	bool find( string name,  TagPtr start, TagPtr end, TagPtr& results , bool search_sub_elements );
	bool find( string name,  TagPair tag, TagPtr& results , bool search_sub_elements );
	bool find_all( string name,  TagPtr start, TagPtr end, TagPtr& results);
	bool find_all( string name,  TagPair tag, TagPtr& results);
	bool find( std::string name, TagPtr start, TagPtr end, TagPair& result, Requerments required, bool in_sub );
	bool find( std::string name, TagPair tag, TagPair& result, Requerments required, bool in_sub );
	string to_string( TagPtr start, TagPtr end );
	string to_string( TagConstPtr start, TagConstPtr end );
	string to_string( const TagPair& pair );
	string to_string( const XMLTags& tags );

	void replace( XMLTags& tags, TagPtr start, TagPtr end, TagPtr new_start, TagPtr new_end );

	void print_blocks( const Context& context );
	void print_machines( const Context& context );
	void print_subs( const Context& context );
	void print_args( const Context& context );
	void print_path( const Context& context );
	void print( const Context& context );


}}}}}



#endif /* SRC_XML_TOKENS_H_ */
