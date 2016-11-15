/*
 * xml_parser_core.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_COGNITAO_PARSER_CORE_H_
#define SRC_COGNITAO_PARSER_CORE_H_


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

struct FUNCTION_IS_NOT_FINISHED{};
#define NOT_FINISHED do{std::cerr<<"EXCEPTION FROM HERE: "<<std::endl<<"\t file: "<<__FILE__<<":"<<__LINE__<<std::endl<<"\t function: "<<__FUNCTION__<<endl; throw FUNCTION_IS_NOT_FINISHED();}while(0);


namespace cognitao{
namespace io {
namespace parser{
namespace core{

using namespace std;
using namespace boost;

struct Node
{
	struct Location{
		size_t open_line;
		size_t open_offset_in_line;
		size_t open_offset_in_file;
		size_t close_line;
		size_t close_offset_in_line;
		size_t close_offset_in_file;
		string source;

		string str()const
		{
			stringstream s;
			s<<"["<<source<<":"<<open_line<<":"<<open_offset_in_line<<","<<open_offset_in_file<<"]";
			return s.str();
		}
	};

	typedef map<string,string> Args;

	string _name;
	string _data;
	Location _location;
	Args _args;
	list<Node> _sub_nodes;

	string str()const
	{
		return _name + " " + location();
	}

	//------- NAME
	const string& name()const
	{
		return _name;
	}

	void set_name(const string& n)
	{
		_name = n;
	}

	//------- DATA
	const string& data()const
	{
		return _data;
	}
	void set_data( const string& d )
	{
		_data = d;
	}

	//-------- ARGS
	const string& operator[](const string& key)const
	{
		return _args.at(key);
	}

	void add_arg( const string& a, const string& v)
	{
		_args[a]=v;
	}
	bool contains( const string& a )const
	{
		return _args.find(a)!=_args.end();
	}
	const Args& args()const
	{
		return _args;
	}

	//-------- LOCATION
	string location()const
	{
		return _location.str();
	}
	void set_location( const Location& loc )
	{
		_location = loc;
	}

	//-------- SUB NODES
	const list<Node>& nodes()const
	{
		return _sub_nodes;
	}
	void add_node(const Node& n)
	{
		_sub_nodes.push_back(n);
	}
	const Node* search( std::string node_name )const
	{
		BOOST_FOREACH( const Node& n, _sub_nodes )
			if( n._name == node_name ) return &n;
		return 0;
	}
};


typedef list<Node> NodesCollection;
typedef NodesCollection::iterator TokenPtr;
typedef NodesCollection::const_iterator TokenConstPtr;

struct MachinesCollection
{
	typedef map<string, Node> Machines;
	Machines machines;
	string start_machine;
};

inline
TokenPtr operator+( const TokenPtr& t, int count )
{
	TokenPtr n = t;
	for(int i=0;i<count;++i) n++;
	return n;
}
inline
TokenPtr operator-( const TokenPtr& t, int count )
{
	TokenPtr n = t;
	for(int i=0;i<count;++i) n--;
	return n;
}

class ParsingError{
public:
	std::string message;
	ParsingError( std::string m = "" ):message(m){}
	template<class T>
	ParsingError operator<<( const T& t ) const
	{
		ParsingError p;
		stringstream s; s<<message<<" "<<t; p.message = s.str(); return p;
	}
};

struct ParsingParameters{
	bool is_case_sensative;
	size_t tab_lenght;

	ParsingParameters()
		: is_case_sensative(false)
		, tab_lenght(4)
	{
	}
};

enum Requerments
{
	Required = true, Optional = false
};

void print( ostream& cout, const Node& node, string tab = "" );
void print_node_to_xml (std::stringstream& xml_stream, const Node& node, string tab ="        " );
void print_machines_collection_to_xml (std::stringstream& xml_stream, cognitao::io::parser::core::MachinesCollection & machines);

}
}

typedef parser::core::MachinesCollection MachinesCollection;
}
}

#define IN_FILE_LOCATION " at "<<__FILE__<<":"<<__LINE__<<""

#endif /* SRC_COGNITAO_PARSER_CORE_H_ */
