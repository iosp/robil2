/*
 * FsmBuilder.h
 *
 *  Created on: Dec 2, 2015
 *      Author: dan
 */

#ifndef SRC_TESTS_FSMBUILDER_H_
#define SRC_TESTS_FSMBUILDER_H_

#include <iostream>
#include <sstream>
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>
#include <cognitao/machine/fsm/Fsm.h>
#include <cognitao/machine/parallel/Parallel.h>

using namespace std;

namespace cognitao {
namespace io {
namespace compiler {
namespace fsm {

class FsmBuilder: public cognitao::io::compiler::MachineBuilder
{

	typedef map<string, cognitao::machine::fsm::State::ptr> StatesTable;

	cognitao::machine::EventProcessor& processor;

	void validate_machine_node( const cognitao::io::parser::core::Node& node );

	void add_transition( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state );

	void add_transitions( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state );

	void build_simple_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state );


	void build_machine_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state, const CompiledMachines& machines );


	void build_parallel_state(
			const cognitao::io::parser::core::Node& node,
			StatesTable& states,
			cognitao::machine::fsm::State::ptr& state,
			const CompiledMachines& machines,
			CompilationObjectsCollector& collector
			);


	int detect_type_of_state( const cognitao::io::parser::core::Node& node );

	void build_state( const cognitao::io::parser::core::Node& state_node, StatesTable& states, const CompiledMachines& machines );


	void init_state(
			const cognitao::io::parser::core::Node& state_node,
			StatesTable& states, const CompiledMachines& machines,
			CompilationObjectsCollector& collector
			);


	void read_states( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::States& states_results );


	string read_node_data( const cognitao::io::parser::core::Node& node );
	string read_node_id( const cognitao::io::parser::core::Node& node );
	void read_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state_results );


	void build_machine( const string& machine_name, const cognitao::io::parser::core::Node& fsm_node, StatesTable& states,cognitao::machine::MachineDefinitionRef& machine );



public:

	FsmBuilder( cognitao::machine::EventProcessor& processor );

	string machine_type()const;

	void build_machine(
				const cognitao::io::parser::core::Node& node,
				const CompiledMachines& machines ,
				CompilationObjectsCollector& collector,
				CompiledMachine& result_machine
				);


};


}
}
}
}

#endif /* SRC_TESTS_FSMBUILDER_H_ */
