#include <cognitao/io/parser/core.h>


namespace cognitao{ namespace io{ namespace parser{ namespace core{

using namespace std;
using namespace boost;

void print( ostream& cout, const Node& node, string tab )
{
	cout<< tab << node.name() <<endl;
	string subtab = tab+"    ";
	if( node.args().empty() == false )
	{
		BOOST_FOREACH( const Node::Args::value_type& a, node.args() )
		{
			cout<< subtab <<a.first<<" = "<<a.second <<endl;
		}
	}
	if( node.nodes().empty() == false )
	{
		BOOST_FOREACH( const NodesCollection::value_type& n, node.nodes() )
		{
			print( cout, n, subtab );
		}
	}
	if( boost::trim_copy(node.data()).empty()==false and node.nodes().empty() )
		cout<< subtab << "DATA:{"<<node.data()<<"}"<<endl;
}



void print_node_to_xml (std::stringstream& xml_stream, const Node& node, string tab)
{
	xml_stream << tab << "<" << node.name();
	std::string subtab = tab + "        ";
	if( node.args().empty() == false )
	{
		BOOST_FOREACH( const Node::Args::value_type& a, node.args() )
		{
			xml_stream<< " "<< a.first<<"=\""<<a.second <<"\"";
		}
		xml_stream << ">" << std::endl;
	} else xml_stream << ">" << std::endl;
	if( node.nodes().empty() == false )
	{
		BOOST_FOREACH( const NodesCollection::value_type& n, node.nodes() )
		{
			print_node_to_xml(xml_stream, n, subtab);
		}
	}
	if( boost::trim_copy(node.data()).empty()==false and node.nodes().empty() )
		xml_stream << subtab << node.data()<<endl;
	xml_stream << tab << "</" << node.name() << ">" << std::endl;
}

void print_machines_collection_to_xml (std::stringstream& xml_stream, cognitao::io::parser::core::MachinesCollection & machines){
	xml_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
	xml_stream << "<tao>" << std::endl;
	xml_stream << "    <machines>" << std::endl;
	BOOST_FOREACH( cognitao::io::parser::core::MachinesCollection::Machines::value_type& v, machines.machines )
	{
		cognitao::io::parser::core::print_node_to_xml(xml_stream, v.second);
	}
	xml_stream << "    <root>" << machines.start_machine << "</root>" << std::endl;
	xml_stream << "    </machines>" << std::endl;
	xml_stream << "</tao>" << std::endl;
}



}}}}

