/*
 * Compiler.h
 *
 *  Created on: Nov 22, 2015
 *      Author: dan
 */

#ifndef SRC_COGNITAO_COMPILER_COMPILER_H_
#define SRC_COGNITAO_COMPILER_COMPILER_H_

#include <cognitao/io/compiler/core.h>

namespace cognitao {
namespace io {
namespace compiler {

typedef cognitao::machine::MachineDefinitionRef MachineRef;

class CompiledData
{
public:
	virtual
	~CompiledData(){}
};

class CompilationObjectsCollector{
public:
	typedef shared_ptr<cognitao::machine::MachineDefinitionRef> Ptr2Machine;
	typedef shared_ptr<CompiledData> Ptr2CompiledData;
	typedef list< shared_ptr<cognitao::machine::MachineDefinitionRef> > ListOfMachines;
	typedef list< shared_ptr<CompiledData> > ListOfCompiledData;

	ListOfMachines machines_definitions;
	ListOfCompiledData compiled_data;
};

typedef CompilationObjectsCollector::Ptr2Machine CompiledMachine;
typedef map<string,CompiledMachine> CompiledMachines;

class MachineBuilder
{
public:
	typedef boost::shared_ptr<MachineBuilder> Ptr;
	virtual ~MachineBuilder(){}

	virtual
	void build_machine(
			const cognitao::io::parser::core::Node& node,
			const CompiledMachines& machines ,
			CompilationObjectsCollector& collector,
			CompiledMachine& result_machine
			)=0;

	virtual
	string machine_type()const=0;
};

typedef map<string,MachineBuilder::Ptr> Builders;

class Compiler {

private:

	CompiledMachines objects;
	Builders builders;


public:


	Compiler();
	virtual ~Compiler();

	virtual
	CompiledMachine compile( const cognitao::io::parser::core::MachinesCollection& machines, cognitao::io::compiler::CompilationObjectsCollector& collector );

	void add_builder( MachineBuilder::Ptr builder );


private:

	void print_objects( ostream& stream )const;
	void create_references_for_all_machines ( const cognitao::io::parser::core::MachinesCollection& machines );
	void build_machine( const cognitao::io::parser::core::Node& node, cognitao::io::compiler::CompilationObjectsCollector& collector, CompiledMachine& machine_ref );
	MachineBuilder& get_builder( const std::string& machine_type , const cognitao::io::parser::core::Node& node);

};

} /* namespace compiler */
} /* namespace io */
} /* namespace cognitao */

#endif /* SRC_COGNITAO_COMPILER_COMPILER_H_ */
