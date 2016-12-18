/*
 * xml_parser_core.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_XML_PARSER_CORE_H_
#define SRC_XML_PARSER_CORE_H_


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <map>
#include <set>
#include <list>
#include <vector>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <exception>

#include "../core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace core{

using namespace std;
using namespace boost;

struct Tag
{
	static string DATA_NAME(){ return "#data"; }
	static string COMENT_NAME(){ return "!--"; }
	static string VERSION_NAME(){ return "?xml"; }

	string name;
	string data;
	size_t open_line;
	size_t open_offset_in_line;
	size_t open_offset_in_file;
	size_t close_line;
	size_t close_offset_in_line;
	size_t close_offset_in_file;
	string source;

	string location()const
	{
		stringstream s;
		s<<"["<<source<<":"<<open_line<<":"<<open_offset_in_line<<","<<open_offset_in_file<<"]";
		return s.str();
	}
	string str()const
	{
		return name + " " + location();
	}
	string xml()const
	{
		if(name == DATA_NAME()) return data;
		stringstream s; s<<"<"<<name<<(data.empty()?"":" ")<<data<<">"; return s.str();
	}
};


typedef list<Tag> XMLTags;
typedef XMLTags::iterator TagPtr;
typedef XMLTags::const_iterator TagConstPtr;

typedef vector<string> PathList;
typedef map<string, XMLTags> SubsTable;
typedef map<string, XMLTags> MachinesTable;
typedef map<string, string> ArgsTable;
typedef map<string, XMLTags> BlocksTable;

inline
TagPtr operator+( const TagPtr& t, int count )
{
	TagPtr n = t;
	for(int i=0;i<count;++i) n++;
	return n;
}
inline
TagPtr operator-( const TagPtr& t, int count )
{
	TagPtr n = t;
	for(int i=0;i<count;++i) n--;
	return n;
}

struct TagPair
{
	TagPtr open;
	TagPtr closed;
	TagPair(){}
	TagPair( TagPtr open, TagPtr closed )
	: open(open), closed(closed)
	{}
	bool single()const{ return open==closed; }
	bool has_body()const{ return not single() and not( open==closed+1 ); }
};


typedef ::cognitao::io::parser::core::ParsingParameters ParsingParameters;

struct Context
{
	PathList path_list;
	SubsTable subs_table;
	BlocksTable blocks_table;
	MachinesTable machines_table;
	ArgsTable args_table;
	string start_machine;
	ParsingParameters parameters;
};

//class ParsingError{
//public:
//	std::string message;
//	ParsingError( std::string m = "" ):message(m){}
//	template<class T>
//	ParsingError operator<<( const T& t ) const
//	{
//		ParsingError p;
//		stringstream s; s<<message<<" "<<t; p.message = s.str(); return p;
//	}
//};
typedef ::cognitao::io::parser::core::ParsingError ParsingError;

enum Requerments
{
	Required = cognitao::io::parser::core::Required,
	Optional = cognitao::io::parser::core::Optional
};
//typedef ::cognitao::io::parser::core::Requerments Requerments;


}}}}}

#define IN_FILE_LOCATION " at "<<__FILE__<<":"<<__LINE__<<""

#endif /* SRC_XML_PARSER_CORE_H_ */
