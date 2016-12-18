/*
 * FsmBuilder.h
 *
 *  Created on: Dec 2, 2015
 *      Author: dan
 */

#ifndef SRC_TESTS_FttBUILDER_H_
#define SRC_TESTS_FttBUILDER_H_

#include <iostream>
#include <sstream>
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>
#include <cognitao/machine/ftt/Ftt.h>
#include <cognitao/machine/parallel/Parallel.h>

using namespace std;

namespace cognitao {
namespace io {
namespace compiler {
namespace ftt {

class FttBuilder: public cognitao::io::compiler::MachineBuilder
{

	typedef map<string, cognitao::machine::ftt::Task::ptr> TasksTable;

	cognitao::machine::EventProcessor& processor;

	void validate_machine_node( const cognitao::io::parser::core::Node& node );

	void add_reaction( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task );

	void add_reactions( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task );

	void search_arguments_of_task( const cognitao::io::parser::core::Node& node, std::string& arguments );

	void build_simple_task( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task );


	void build_machine_task( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task, const CompiledMachines& machines );


	void build_parallel_task(
			const cognitao::io::parser::core::Node& node,
			TasksTable& tasks,
			cognitao::machine::ftt::Task::ptr& task,
			const CompiledMachines& machines,
			CompilationObjectsCollector& collector
			);


	int detect_type_of_task( const cognitao::io::parser::core::Node& node );

	void build_task( const cognitao::io::parser::core::Node& task_node, TasksTable& tasks, const CompiledMachines& machines );


	void init_task(
			const cognitao::io::parser::core::Node& task_node,
			TasksTable& tasks, const CompiledMachines& machines,
			CompilationObjectsCollector& collector
			);


	void read_tasks( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Tasks& tasks_results );


	string read_node_data( const cognitao::io::parser::core::Node& node );
	string read_node_id( const cognitao::io::parser::core::Node& node );
	string read_node_property( const cognitao::io::parser::core::Node& node, const string& prop_name , bool* error = 0);
	void read_task( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task_results );


	void build_machine( const string& machine_name, const cognitao::io::parser::core::Node& fsm_node, TasksTable& tasks,cognitao::machine::MachineDefinitionRef& machine );



public:

	FttBuilder( cognitao::machine::EventProcessor& processor );

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
