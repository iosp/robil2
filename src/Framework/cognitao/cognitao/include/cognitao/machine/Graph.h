/*
 * Graph.h
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_GRAPH_H_
#define INCLUDE_COGNITAO_GRAPH_H_

#include <map>
#include <list>
#include <set>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;

namespace cognitao {
namespace machine {
namespace graph {
typedef map<string, string> Properties;

class Graph;
struct Edge;

struct GraphElement
{
	Properties properties;

	string operator[](const string& name)const{ if(properties.find(name)==properties.end()) return ""; return properties.at(name); }
	string& operator[](const string& name){ return properties[name]; }

	string property(const string& name)const{ if(properties.find(name)==properties.end()) return ""; return properties.at(name); }
	string& property(const string& name){ return properties[name]; }

	bool contains( const string& name )const { return properties.find(name)!=properties.end(); }

	string id()const{ return (*this)["id"]; }
	void id( string i ){ (*this)["id"] = i; }

	string type()const{ return (*this)["type"]; }
	void type( string t ){ (*this)["type"] = t; }

};

struct Node: public GraphElement
{
	typedef shared_ptr<Node> ptr;

	//Graph& graph;

	set<Edge*> input;
	set<Edge*> output;

	Node( Graph& g, const string& name )
		//:graph(g)
	{
		(*this)["id"] = name;
	}

};


struct Edge: public GraphElement
{
	typedef shared_ptr<Edge> ptr;

	//Graph& graph;

	Node* source;
	Node* target;


	Edge( Graph& g, Node* s, Node* t, const string& id, const string& type)
		//:graph(g),
	:source(s), target(t)
	{
		(*this)["id"] = id;
		(*this)["type"] = type;
	}

};


class Graph {

public:



public:
	Graph();
	virtual ~Graph();

	typedef map<string, Node::ptr> Nodes;
	Nodes nodes;
	typedef map<string, Edge::ptr> Edges;
	Edges edges;

	 Node* node(const string& id)const
	{
		if( nodes.find(id)==nodes.end() ) return 0;
		return nodes.at(id).get();
	}
	Node* node(const string& id)
	{
		if( nodes.find(id)==nodes.end() )
		{
			Node* n = new Node( *this, id );
			nodes[id] = Node::ptr(n);
		}
		return nodes[id].get();
	}

	Edge* edge(const string& id)const
	{
		if( edges.find(id)==edges.end() ) return 0;
		return edges.at(id).get();
	}
	Edge* edge(Node* s, Node* d, const string& id, const string& type)
	{
		if( edges.find(id)==edges.end() )
		{
			Edge* e = new Edge(*this, s, d, id, type);
			edges[id] = Edge::ptr(e);
			s->output.insert(e);
			d->input.insert(e);
		}
		return edges[id].get();
	}

	Graph& operator+=(const Graph& g)
	{
		nodes.insert(g.nodes.begin(), g.nodes.end());
		edges.insert(g.edges.begin(), g.edges.end());
		return *this;
	}

	bool is_root( Node* node)const
	{
		BOOST_FOREACH( Edges::value_type v, edges )
		{
			if( v.second->target == node ) return false;
		}
		return true;
	}
	Node* search_root()const
	{

		BOOST_FOREACH( Nodes::value_type v, nodes )
		{
			if( is_root( v.second.get() ) ) return v.second.get();
		}
		return 0;
	}

	set<Edge*> search_edges( Node* s, Node* t )const
	{
		set<Edge*> res;
		BOOST_FOREACH( Edges::value_type v, edges )
		{
			if( v.second->source == s and v.second->target == t ) res.insert(v.second.get());
		}
		return res;
	}
	set<Edge*> search_edges_income( Node* t )const
	{
		set<Edge*> res;
		BOOST_FOREACH( Edges::value_type v, edges )
		{
			if( v.second->target == t ) res.insert(v.second.get());
		}
		return res;
	}
	set<Edge*> search_edges_outcome( Node* s )const
	{
		set<Edge*> res;
		BOOST_FOREACH( Edges::value_type v, edges )
		{
			if( v.second->source == s ) res.insert(v.second.get());
		}
		return res;
	}

	Graph add_id_prefix( std::string pref )const
	{
		Graph g = *this;
		BOOST_FOREACH( Nodes::value_type& v, g.nodes )
		{
			v.second->property( "id" ) = pref + v.second->property( "id" );
		}
		BOOST_FOREACH( Edges::value_type v, edges )
		{
			v.second->property( "id" ) = pref + v.second->property( "id" );
		}
		return g;
	}
};

class GraphPrinter{
public:
	virtual
	~GraphPrinter(){}

	virtual
	ostream& print(ostream& cout, Node* node) = 0;
};

class SimplePrinter;
class SimplePrinterMachine
{
public:
	const SimplePrinter* printer;

	virtual
	~SimplePrinterMachine(){}

	virtual
	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const=0;

//	virtual
//	ostream& print(ostream& cout, Edge* e, string tab="", set<Node*>* visited = 0)const=0;
//
//	virtual
//	ostream& print(ostream& cout, const Properties& prop)const=0;

};

class SimplePrinter: public GraphPrinter
{
public:

	class Exception_StartNodeIsNotMachine{};
	class Exception_MachineNameIsNotFound{};

	map<string, boost::shared_ptr<SimplePrinterMachine> > printers;

	virtual
	ostream& print(ostream& cout, Node* node);
	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const;

};

class SimplePrinterForFsm: public SimplePrinterMachine
{
public:

	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const;
	ostream& print_state_for_trans(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const;
	ostream& print_state_for_defs(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const;

	ostream& print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;
	ostream& print_state_for_trans(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;
	ostream& print_state_for_subs(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;

	ostream& print(ostream& cout, const Properties& prop)const;

};


class SimplePrinterForFtt: public SimplePrinterMachine
{
public:

	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const;
	ostream& print_state_for_trans(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const;
	ostream& print_state_for_defs(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const;

	ostream& print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;
	ostream& print_state_for_trans(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;
	ostream& print_state_for_subs(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;

	ostream& print(ostream& cout, const Properties& prop)const;

};


class SimplePrinterForParallel: public SimplePrinterMachine
{
public:

	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const;
	ostream& print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const;
	ostream& print(ostream& cout, const Properties& prop)const;
};

class XmlPrinter: public GraphPrinter
{
public:
	virtual
	ostream& print(ostream& cout, Node* node)
	{
		set<Node*> inst_visited;

		return print( cout, node, "", true, &inst_visited );
	}

private:
	ostream& print(ostream& cout, Node* node, string tab, bool ftab = true, set<Node*>* visited = 0)const;

	ostream& print(ostream& cout, Edge* e, string tab="", set<Node*>* visited = 0)const;

	ostream& print(ostream& cout, const Properties& prop)const;

};


}
} /* namespace machine */
} /* namespace cognitao */

#endif /* INCLUDE_COGNITAO_GRAPH_H_ */
