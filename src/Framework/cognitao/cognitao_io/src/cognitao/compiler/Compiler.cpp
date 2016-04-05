/*
 * Compiler.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: dan
 */

#include <cognitao/io/compiler/Compiler.h>


namespace cognitao {
namespace io {
namespace compiler {

Compiler::Compiler() {

}

Compiler::~Compiler() {

}


void Compiler::print_objects( ostream& stream )const
{

	BOOST_FOREACH( const CompiledMachines::value_type& m, objects )
	{

		stream << "===== "<<m.first <<( m.second and m.second->machine ?" is created":" is not created")<< " =======" <<endl;

	}
}

void Compiler::create_references_for_all_machines ( const cognitao::io::parser::core::MachinesCollection& machines )
{
	BOOST_FOREACH( const cognitao::io::parser::core::MachinesCollection::Machines::value_type& m, machines.machines )
	{
		objects[ m.first ] = CompiledMachine(new cognitao::machine::MachineDefinitionRef );
	}

}

void Compiler::add_builder( MachineBuilder::Ptr builder )
{
	assert( builder.get() );

	builders[ builder->machine_type() ] = builder;
}

MachineBuilder& Compiler::get_builder( const std::string& machine_type, const cognitao::io::parser::core::Node& node )
{
	Builders::iterator i_builder = builders.find(machine_type);
	if(builders.end()==i_builder) throw CompilerError()<<"Builder for machine type "<<machine_type<<" does not found. "<<node.str()<<IN_FILE_LOCATION;

	MachineBuilder::Ptr builder = i_builder->second;
	if(not builder) throw CompilerError()<<"Builder for machine type "<<machine_type<<" does not found. "<<node.str()<<IN_FILE_LOCATION;

	return *builder;
}

void Compiler::build_machine( const cognitao::io::parser::core::Node& node, cognitao::io::compiler::CompilationObjectsCollector& collector,  CompiledMachine& m )
{
	//CompiledMachine m;

	if( node.name() != "machine" ) 									  throw CompilerError()<<"Wrong token name for machine building. "<<node.str()<<IN_FILE_LOCATION;
	if( node.contains("type")==false or node.contains("id")==false )  throw CompilerError()<<"Token does not contain required attributes (type and id). "<<node.str()<<IN_FILE_LOCATION;

	string machine_type = node["type"];

	get_builder(machine_type, node).build_machine( node, objects, collector , m);
}

cognitao::io::compiler::CompiledMachine Compiler::compile( const cognitao::io::parser::core::MachinesCollection& machines , cognitao::io::compiler::CompilationObjectsCollector& collector)
{

	create_references_for_all_machines( machines );
	print_objects( cout );

	BOOST_FOREACH( const cognitao::io::parser::core::MachinesCollection::Machines::value_type& m, machines.machines )
	{
		CompiledMachine object = objects[ m.first ];
		build_machine( m.second , collector , object );
		if( not object or not object->machine ) throw cognitao::io::compiler::CompilerError()<<"Object is empty. No Compilation results for "<<m.first<<" "<<m.second.str()<<IN_FILE_LOCATION;

		//assert(0);

		//objects[ m.first ] = object;
	}

	BOOST_FOREACH( CompiledMachines::value_type& v, objects )
	{
		collector.machines_definitions.push_back( v.second );
	}

	// RETURN START MACHINE
	return objects[ machines.start_machine ];
}






} /* namespace compiler */
} /* namespace io */
} /* namespace cognitao */
