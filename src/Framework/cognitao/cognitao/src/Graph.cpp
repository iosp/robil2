/*
 * Graph.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#include <cognitao/machine/Graph.h>

namespace cognitao {
namespace machine {
namespace graph{

Graph::Graph() {
	// NOTE: Auto-generated constructor stub

}

Graph::~Graph() {
	// NOTE: Auto-generated destructor stub
}


//================== PRINTERS ===================================

ostream& SimplePrinter::print(ostream& cout, Node* node)
{
	set<Node*> inst_visited;
	return print( cout, node, "", true, &inst_visited);
}

ostream& SimplePrinter::print(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	if( node->contains("machine") )
	{
		if(printers.find(node->property("machine")) == printers.end() ) throw Exception_MachineNameIsNotFound();

		printers.at(node->property("machine"))->printer = this;
		return printers.at(node->property("machine"))->print( cout, node, tab, ftab, visited);
	}
	throw Exception_StartNodeIsNotMachine();
	return cout;
}

//-------------------- FSM

ostream& SimplePrinterForFsm::print(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{

	if( visited->find(node)!=visited->end() )
	{
		return cout<<tab<<node->id()<<" ---> ";
	}

	if( node->contains("machine") and node->property("machine")!="fsm" )
	{
		return printer->print( cout, node, tab, ftab, visited);
	}

	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"{";
		print(cout, node->properties) << endl;
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "contains" ) continue;
			print( cout , e , tab+"   ", visited ) << endl;
		}
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "start" ) continue;
			print_state_for_trans( cout , e , tab+"   ", visited ) << endl;
		}
	cout<<tab<<"}";
	return cout;
}
ostream& SimplePrinterForFsm::print_state_for_trans(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	return cout<<" "<<node->id()<<" --->";
	return cout;
}
ostream& SimplePrinterForFsm::print_state_for_defs(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	if( node->contains("machine") and node->property("machine")!="fsm" )
	{
		return printer->print( cout, node, tab, ftab, visited);
	}

	if( visited->find(node)!=visited->end())
	{
		return cout<<tab<<node->id()<<" ---> ";
	}
	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"{";
	print(cout, node->properties) << endl;

		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "fsm_transition" ) continue;
			print_state_for_trans( cout , e , tab+"   ", visited ) << endl;
		}
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") == "fsm_transition" ) continue;
			print_state_for_subs( cout , e , tab+"   ", visited ) << endl;
		}

	cout<<tab<<"}";
	return cout;
}
ostream& SimplePrinterForFsm::print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " <<endl;
	print_state_for_defs( cout , e->target, tab+"   " , true, visited);
	return cout;
}
ostream& SimplePrinterForFsm::print_state_for_trans(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " ;
	print_state_for_trans( cout , e->target, tab+"   " , true, visited );
	return cout;
}
ostream& SimplePrinterForFsm::print_state_for_subs(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " <<endl;
	print( cout , e->target, tab+"   " , true, visited );
	return cout;
}

ostream& SimplePrinterForFsm::print(ostream& cout, const Properties& prop)const
{
	cout<<"[";
	BOOST_FOREACH( const Properties::value_type& p, prop )
	{
		cout<<" "<<p.first<<"="<<p.second;
	}
	return cout<<" ]";
}


ostream& SimplePrinterForParallel::print(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	if( visited->find(node)!=visited->end() )
	{
		return cout<<tab<<node->id()<<" ---> ";
	}

	if( node->contains("machine") and node->property("machine")!="parallel" )
	{
		return printer->print( cout, node, tab, ftab, visited);
	}

	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"{";
		print(cout, node->properties) << endl;
		BOOST_FOREACH( Edge* e, node->output )
		{
			print( cout , e , tab+"   ", visited ) << endl;
		}
	cout<<tab<<"}";
	return cout;
}
ostream& SimplePrinterForParallel::print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " <<endl;
	print( cout , e->target, tab+"   " , true, visited);
	return cout;
}

ostream& SimplePrinterForParallel::print(ostream& cout, const Properties& prop)const
{
	cout<<"[";
	BOOST_FOREACH( const Properties::value_type& p, prop )
	{
		cout<<" "<<p.first<<"="<<p.second;
	}
	return cout<<" ]";
}


//-------------------- FSM

ostream& SimplePrinterForFtt::print(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{

	if( visited->find(node)!=visited->end() )
	{
		return cout<<tab<<node->id()<<" ---> ";
	}

	if( node->contains("machine") and node->property("machine")!="ftt" )
	{
		return printer->print( cout, node, tab, ftab, visited);
	}

	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"{";
		print(cout, node->properties) << endl;
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "contains" ) continue;
			print( cout , e , tab+"   ", visited ) << endl;
		}
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "start" ) continue;
			print_state_for_trans( cout , e , tab+"   ", visited ) << endl;
		}
	cout<<tab<<"}";
	return cout;
}
ostream& SimplePrinterForFtt::print_state_for_trans(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	return cout<<" "<<node->id()<<" --->";
	return cout;
}
ostream& SimplePrinterForFtt::print_state_for_defs(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	if( node->contains("machine") and node->property("machine")!="ftt" )
	{
		return printer->print( cout, node, tab, ftab, visited);
	}

	if( visited->find(node)!=visited->end())
	{
		return cout<<tab<<node->id()<<" ---> ";
	}
	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"{";
	print(cout, node->properties) << endl;

		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") != "ftt_reaction" ) continue;
			print_state_for_trans( cout , e , tab+"   ", visited ) << endl;
		}
		BOOST_FOREACH( Edge* e, node->output )
		{
			if( e->property("type") == "ftt_reaction" ) continue;
			print_state_for_subs( cout , e , tab+"   ", visited ) << endl;
		}

	cout<<tab<<"}";
	return cout;
}
ostream& SimplePrinterForFtt::print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " <<endl;
	print_state_for_defs( cout , e->target, tab+"   " , true, visited);
	return cout;
}
ostream& SimplePrinterForFtt::print_state_for_trans(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " ;
	print_state_for_trans( cout , e->target, tab+"   " , true, visited );
	return cout;
}
ostream& SimplePrinterForFtt::print_state_for_subs(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"-";
	print(cout, e->properties) <<"-> " <<endl;
	print( cout , e->target, tab+"   " , true, visited );
	return cout;
}

ostream& SimplePrinterForFtt::print(ostream& cout, const Properties& prop)const
{
	cout<<"[";
	BOOST_FOREACH( const Properties::value_type& p, prop )
	{
		cout<<" "<<p.first<<"="<<p.second;
	}
	return cout<<" ]";
}


//--------------------------------- XML -----------------------------------


ostream& XmlPrinter::print(ostream& cout, Node* node, string tab, bool ftab, set<Node*>* visited)const
{
	if( visited->find(node)!=visited->end() )
	{
		return cout<<tab<<"<node id=\""<<node->id()<<" />";
	}
	visited->insert(node);

	if(ftab) cout<<tab;
	cout<<"<node ";
	print(cout, node->properties) <<">"<< endl;
	BOOST_FOREACH( Edge* e, node->output )
	{
		print( cout , e , tab+"  ", visited) << endl;
	}
	cout<<tab<<"</node>";
	return cout;
}

ostream& XmlPrinter::print(ostream& cout, Edge* e, string tab, set<Node*>* visited)const
{
	cout<<tab<<"<edge ";
	print(cout, e->properties) <<"> " <<endl;

	print( cout , e->target, tab+"  " , true, visited ) << endl;

	cout<<tab<<"</edge>";
	return cout;
}

ostream& XmlPrinter::print(ostream& cout, const Properties& prop)const
{
	cout<<"";
	BOOST_FOREACH( const Properties::value_type& p, prop )
	{
		cout<<" "<<p.first<<"=\""<<p.second<<"\"";
	}
	return cout<<"";
}












}
} /* namespace machine */
} /* namespace cognitao */
