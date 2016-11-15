/*
 * FttBuilder.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: dan
 */

#include <cognitao/io/compiler/ftt/FttBuilder.h>

namespace cognitao {
namespace io {
namespace compiler {
namespace ftt {

class FttCompilationData: public cognitao::io::compiler::CompiledData
{
public:
	list<cognitao::machine::ftt::Task::ptr> tasks;
};

void FttBuilder::validate_machine_node( const cognitao::io::parser::core::Node& node )
{
	if( node.contains("type") == false or node["type"] != "ftt" ) throw cognitao::io::compiler::CompilerError()<<"Type of machine is wrong. "<<node.str()<<IN_FILE_LOCATION;
	if( node.contains("id") == false or boost::trim_copy(node["id"]).empty() ) throw cognitao::io::compiler::CompilerError()<<"Id of machine is wrong. "<<node.str()<<IN_FILE_LOCATION;
}

void FttBuilder::add_reaction( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task )
{
	cognitao::machine::Events triggers, outputs;


	if( node.contains("on_event") )
	{
		string event_name = node["on_event"];
		triggers+=cognitao::machine::Event( event_name );
	}else{
		throw cognitao::io::compiler::CompilerError()<<"Can not find 'on_event' property of reaction"<<node.str()<<IN_FILE_LOCATION;
	}


	if( node.contains("send") )
	{
		string send_event_name = node["send"];
		outputs +=cognitao::machine::Event( send_event_name );
	}

	if( node.contains("target") )
	{
		string target_name = node["target"];

		if( tasks.find( target_name ) == tasks.end() )
			throw cognitao::io::compiler::CompilerError()<<"Target task name ["<<target_name<<"] is wrong, because such task has not been defined "<<node.str()<<IN_FILE_LOCATION;

		cognitao::machine::ftt::Task::ptr target = tasks.at( target_name );

		task->add_condition( cognitao::machine::ftt::Condition::ptr(
				new cognitao::machine::ftt::Reaction( triggers, outputs, *target )
		) );

	}else{
		throw cognitao::io::compiler::CompilerError()<<"Can not find 'target' property of reaction"<<node.str()<<IN_FILE_LOCATION;
	}

}

void FttBuilder::add_reactions( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task )
{
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() == "reaction" )
		{
			add_reaction( n, tasks, task );
		}
	}
}

void FttBuilder::search_arguments_of_task( const cognitao::io::parser::core::Node& node, std::string& arguments )
{
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() != "arguments" ) continue;

		arguments = n.data();

		break;
	}
}

void FttBuilder::build_simple_task( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task )
{
	std::cout<<"[BUILDER] create simple task ..."<<std::endl;
	bool err=false;

	string task_id = read_node_id(node);
	std::cout<<"[BUILDER] .... id = "<<task_id<<std::endl;

	string behavior_id = read_node_property(node, "behavior", &err);
	if(err) behavior_id = task_id;
	std::cout<<"[BUILDER] .... behavior= "<<behavior_id<<std::endl;

	string arguments;
	search_arguments_of_task( node, arguments );

	std::cout<<"[BUILDER] .... create simple task "<<task_id<<", "<<behavior_id<<std::endl;
	task = cognitao::machine::ftt::Task::ptr ( new cognitao::machine::ftt::Task(task_id, behavior_id, arguments) );
}

void FttBuilder::build_machine_task(
		const cognitao::io::parser::core::Node& node,
		TasksTable& tasks,
		cognitao::machine::ftt::Task::ptr& task,
		const CompiledMachines& machines
	 )
{
	std::cout<<"[BUILDER] create machine task ..."<<std::endl;
	CompiledMachine _m;
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() != "machine" ) continue;
		string n_id = read_node_id(n);

		if( machines.find(n_id) == machines.end() ) throw CompilerError()<<"Machine wrong name. "<<n.str()<<IN_FILE_LOCATION;

		_m = machines.at(n_id);
		break;
	}
	if( not _m ) throw CompilerError()<<"Machine tag not found in "<<node.str()<<IN_FILE_LOCATION;

	bool err=false;

	string task_id = read_node_id(node);
	std::cout<<"[BUILDER] .... id = "<<task_id<<std::endl;

	string behavior_id = read_node_property(node, "behavior", &err);
	if(err) behavior_id = task_id;
	std::cout<<"[BUILDER] .... behavior= "<<behavior_id<<std::endl;

	string arguments;
	search_arguments_of_task( node, arguments );

	std::cout<<"[BUILDER] .... create machine task "<<task_id<<", "<<behavior_id<<std::endl;
	task = cognitao::machine::ftt::Task::ptr ( new cognitao::machine::ftt::TaskMachine(task_id, behavior_id, arguments, *_m) );
}


namespace {

const string stop_condition_name__while_all_run("all");
static
int stop_condition__while_all_run( const cognitao::machine::parallel::Statistic& stat )
{
	cout<<"ALL STAT: "<<stat.total_count<<", "<<stat.successed_count<<", "<<stat.failures_count<<endl;
	return stat.successed_count+stat.failures_count > 0; // stop if somebody is stopped
}

const string stop_condition_name__while_any_run("any");
static
int stop_condition__while_any_run( const cognitao::machine::parallel::Statistic& stat )
{
	cout<<"ANY STAT: "<<stat.total_count<<", "<<stat.successed_count<<", "<<stat.failures_count<<endl;
	return stat.successed_count+stat.failures_count == stat.total_count; // stop if everybody are stopped
}

}


void FttBuilder::build_parallel_task(
		const cognitao::io::parser::core::Node& node,
		TasksTable& tasks,
		cognitao::machine::ftt::Task::ptr& task,
		const CompiledMachines& machines,
		CompilationObjectsCollector& collector
	 )
{

	std::cout<<"[BUILDER] create multi-machine (parallel) task ..."<<std::endl;
	cognitao::machine::parallel::MachineList m_machines;
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() != "machine" ) continue;
		string n_id = read_node_id(n);

		if( machines.find(n_id) == machines.end() ) throw CompilerError()<<"Machine wrong name. "<<n.str()<<IN_FILE_LOCATION;

		m_machines.add( *(machines.at(n_id)) );
	}


	bool err=false;

	string task_id = read_node_id(node);
	std::cout<<"[BUILDER] .... id = "<<task_id<<std::endl;

	string behavior_id = read_node_property(node, "behavior", &err);
	if(err) behavior_id = task_id;
	std::cout<<"[BUILDER] .... behavior= "<<behavior_id<<std::endl;

	string parallel_policy = read_node_property(node, "parallel", &err);
	boost::trim(parallel_policy);
	boost::to_lower(parallel_policy);
	if(err) parallel_policy = stop_condition_name__while_all_run;
	std::cout<<"[BUILDER] .... parallel= "<<parallel_policy<<std::endl;

	string arguments;
	search_arguments_of_task( node, arguments );

	typedef int (*parallel_stop_condition)( const cognitao::machine::parallel::Statistic& stat );
	parallel_stop_condition stop_condition = stop_condition__while_all_run;
	if( parallel_policy == stop_condition_name__while_all_run ) stop_condition = stop_condition__while_all_run;
	if( parallel_policy == stop_condition_name__while_any_run ) stop_condition = stop_condition__while_any_run;

	CompiledMachine _m( new cognitao::machine::MachineDefinitionRef(
			new cognitao::machine::parallel::ParallelDefinition(
					parallel_policy,
					processor,
					m_machines,
					stop_condition
			)
	) );
	collector.machines_definitions.push_back( _m );

	std::cout<<"[BUILDER] .... create machine task "<<task_id<<", "<<behavior_id<<std::endl;
	task = cognitao::machine::ftt::Task::ptr ( new cognitao::machine::ftt::TaskMachine(task_id, behavior_id, arguments, *_m) );

}

int FttBuilder::detect_type_of_task( const cognitao::io::parser::core::Node& node )
{
	int machine_counter = 0;
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() == "machine" ) machine_counter++;
	}
	if( machine_counter == 0 ) return 0;
	if( machine_counter == 1 ) return 1;
	return 2;
}

void FttBuilder::build_task( const cognitao::io::parser::core::Node& task_node, TasksTable& tasks, const CompiledMachines& machines )
{
	string task_id;
	try{
		task_id = read_node_id( task_node );
	}catch(const CompilerError& err ){ throw err<<"Task name is wrong. "<<task_node.str()<<IN_FILE_LOCATION; }

	add_reactions( task_node, tasks, tasks[task_id] );
}

void FttBuilder::init_task(
		const cognitao::io::parser::core::Node& task_node,
		TasksTable& tasks, const CompiledMachines& machines,
		CompilationObjectsCollector& collector
		)
{
	string task_id;
	try{
		task_id = read_node_id( task_node );
	}catch(const CompilerError& err ){ throw err<<"Task name is wrong. "<<task_node.str()<<IN_FILE_LOCATION; }

	int task_type = detect_type_of_task( task_node );

	switch( task_type )
	{
		case 0: build_simple_task( task_node, tasks, tasks[task_id] ); break;
		case 1: build_machine_task( task_node, tasks, tasks[task_id], machines ); break;
		case 2: build_parallel_task( task_node, tasks, tasks[task_id], machines, collector ); break;
	};
	cout<<"   "<<"task "<<task_id<<" has "<<(tasks[task_id]?"been defined":"not been defined")<<endl;
}

void FttBuilder::read_tasks( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Tasks& tasks_results )
{
	BOOST_FOREACH( const cognitao::io::compiler::Node& sub_node, node.nodes() )
	{
		cognitao::machine::ftt::Task::ptr task;
		if( sub_node.name() == "task" )
		{
			read_task( sub_node, tasks, task );

			tasks_results += task;
		}
	}
}

string FttBuilder::read_node_data( const cognitao::io::parser::core::Node& node )
{
	string name = node.data();
	boost::trim(name);
	if( name.empty() ) throw cognitao::io::compiler::CompilerError()<<"Start task name is wrong. "<<node.str()<<IN_FILE_LOCATION;
	return name;
}
string FttBuilder::read_node_id( const cognitao::io::parser::core::Node& node )
{
	return read_node_property(node, "id");
}

string FttBuilder::read_node_property( const cognitao::io::parser::core::Node& node, const string& prop_name, bool* error_ptr )
{
	try{
		string name = node[prop_name];
		boost::trim(name);
		if( name.empty() )
		{
			if(not error_ptr) throw cognitao::io::compiler::CompilerError()<<"task "<<prop_name<<" is wrong. "<<node.str()<<IN_FILE_LOCATION;
			*error_ptr = true;
		}
		if(error_ptr) *error_ptr=false;
		return name;
	}catch(...){
		if(not error_ptr) throw cognitao::io::compiler::CompilerError()<<"task "<<prop_name<<" is wrong. "<<node.str()<<IN_FILE_LOCATION;
		*error_ptr=true; return "";
	}
}

void FttBuilder::read_task( const cognitao::io::parser::core::Node& node, TasksTable& tasks, cognitao::machine::ftt::Task::ptr& task_results )
{
	string task_name;
	try{
		task_name = read_node_data( node );
	}catch(const CompilerError& err ){ throw err<<"Start task name is wrong. "<<node.str()<<IN_FILE_LOCATION; }

	if( tasks.find( task_name ) == tasks.end() )
		throw cognitao::io::compiler::CompilerError()<<"Start task name ["<<task_name<<"] is wrong, because such task has not been defined "<<node.str()<<IN_FILE_LOCATION;

	cognitao::machine::ftt::Task::ptr taskRef = tasks.at( task_name );
	if( not taskRef ) throw cognitao::io::compiler::CompilerError()<<"task "<<task_name<<" does not defined."<<node.str()<<IN_FILE_LOCATION;

	task_results = taskRef;
}

void FttBuilder::build_machine( const string& machine_name, const cognitao::io::parser::core::Node& ftt_node, TasksTable& tasks, cognitao::machine::MachineDefinitionRef& machine )
{

	cognitao::machine::ftt::Tasks all_tasks;
	BOOST_FOREACH( TasksTable::value_type& v, tasks )
	{
		all_tasks += v.second;
	}

	cognitao::machine::ftt::Tasks success_tasks;
	cognitao::machine::ftt::Tasks fail_tasks;

	cognitao::machine::ftt::Task::ptr start_task;

	BOOST_FOREACH( const cognitao::io::compiler::Node& sub_node, ftt_node.nodes() )
	{
		if( sub_node.name() == "success" )	read_tasks( sub_node, tasks, success_tasks );
		if( sub_node.name() == "fail" ) 	read_tasks( sub_node, tasks, success_tasks );
		if( sub_node.name() == "start" ) 	read_task ( sub_node, tasks, start_task);
	}

	if( not start_task ) throw cognitao::io::compiler::CompilerError()<<"Start task name is not defined. "<<ftt_node.str()<<IN_FILE_LOCATION;

	machine = cognitao::machine::MachineDefinitionRef(
			new cognitao::machine::ftt::FttDefinition(
					machine_name, processor,
					all_tasks, *start_task,
					success_tasks , fail_tasks
			)
	);
}

FttBuilder::FttBuilder( cognitao::machine::EventProcessor& processor )
: processor( processor )
{

}

string FttBuilder::machine_type()const
{
	return "ftt";
}

void FttBuilder::build_machine(
		const cognitao::io::parser::core::Node& node,
		const CompiledMachines& machines ,
		CompilationObjectsCollector& collector,
		CompiledMachine& machine
		)
{
	validate_machine_node( node );
	cout<<" Build FTT machine from token node "<< node["id"] <<" "<< node.str() <<endl;

	TasksTable tasks;
	BOOST_FOREACH( const cognitao::io::compiler::Node& machine_sub_node , node.nodes() )
	{
		if( machine_sub_node.name() == "tasks" )
		{
			BOOST_FOREACH( const cognitao::io::compiler::Node& tasks_node , machine_sub_node.nodes() )
			{
				cout<<"Task "<<tasks_node.name()<<" "<<tasks_node["id"]<<" for init"<<endl;
				init_task( tasks_node, tasks, machines, collector );
			}
			BOOST_FOREACH( const cognitao::io::compiler::Node& tasks_node , machine_sub_node.nodes() )
			{
				cout<<"Task "<<tasks_node.name()<<" "<<tasks_node["id"]<<" for build"<<endl;
				build_task( tasks_node, tasks, machines);
			}
		}else if( machine_sub_node.name() == "ftt" )
		{
			cout<<"Ftt initialization information: "<<machine_sub_node.name()<<""<<endl;
			build_machine( node["id"], machine_sub_node, tasks, *machine );


		}
	}

	shared_ptr<FttCompilationData> fcd( new FttCompilationData );
	BOOST_FOREACH( TasksTable::value_type& v, tasks )
	{
		fcd->tasks.push_back( v.second );
	}
	collector.compiled_data.push_back(fcd);

}

}
}
}
}



