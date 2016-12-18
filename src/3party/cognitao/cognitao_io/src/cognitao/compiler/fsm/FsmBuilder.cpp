/*
 * FsmBuilder.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: dan
 */

#include <cognitao/io/compiler/fsm/FsmBuilder.h>

namespace cognitao {
namespace io {
namespace compiler {
namespace fsm {

class FsmCompilationData: public cognitao::io::compiler::CompiledData
{
public:
	list<cognitao::machine::fsm::State::ptr> states;
};

void FsmBuilder::validate_machine_node( const cognitao::io::parser::core::Node& node )
{
	if( node.contains("type") == false or node["type"] != "fsm" ) throw cognitao::io::compiler::CompilerError()<<"Type of machine is wrong. "<<node.str()<<IN_FILE_LOCATION;
	if( node.contains("id") == false or boost::trim_copy(node["id"]).empty() ) throw cognitao::io::compiler::CompilerError()<<"Id of machine is wrong. "<<node.str()<<IN_FILE_LOCATION;
}

void FsmBuilder::add_transition( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state )
{
	cognitao::machine::Events triggers, outputs;


	if( node.contains("on_event") )
	{
		string event_name = node["on_event"];
		triggers+=cognitao::machine::Event( event_name );
	}else{
		throw cognitao::io::compiler::CompilerError()<<"Can not find 'on_event' property of transition"<<node.str()<<IN_FILE_LOCATION;
	}


	if( node.contains("send") )
	{
		string send_event_name = node["send"];
		outputs +=cognitao::machine::Event( send_event_name );
	}

	if( node.contains("target") )
	{
		string target_name = node["target"];

		if( states.find( target_name ) == states.end() )
			throw cognitao::io::compiler::CompilerError()<<"Target state name ["<<target_name<<"] is wrong, because such state has not been defined "<<node.str()<<IN_FILE_LOCATION;

		cognitao::machine::fsm::State::ptr target = states.at( target_name );

		state->add_condition( cognitao::machine::fsm::Condition::ptr(
				new cognitao::machine::fsm::Transition( triggers, outputs, *target )
		) );

	}else{
		throw cognitao::io::compiler::CompilerError()<<"Can not find 'target' property of transition"<<node.str()<<IN_FILE_LOCATION;
	}

}

void FsmBuilder::add_transitions( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state )
{
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() == "transit" )
		{
			add_transition( n, states, state );
		}
	}
}

void FsmBuilder::build_simple_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state )
{
	string state_id = read_node_id(node);
	state = cognitao::machine::fsm::State::ptr ( new cognitao::machine::fsm::State(state_id) );
}

void FsmBuilder::build_machine_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state, const CompiledMachines& machines )
{
	CompiledMachine _m;
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() != "machine" ) continue;
		string n_id = read_node_id(n);

		if( machines.find(n_id) == machines.end() ) throw CompilerError()<<"Machine wring name. "<<n.str()<<IN_FILE_LOCATION;

		_m = machines.at(n_id);
		break;
	}

	string state_id = read_node_id(node);
	state = cognitao::machine::fsm::State::ptr ( new cognitao::machine::fsm::StateMachine(state_id, *_m) );
}


namespace {

static
int stop_condition( const cognitao::machine::parallel::Statistic& stat )
{
	cout<<"STAT: "<<stat.total_count<<", "<<stat.successed_count<<", "<<stat.failures_count<<endl;
	return 1;
}

}


void FsmBuilder::build_parallel_state(
		const cognitao::io::parser::core::Node& node,
		StatesTable& states,
		cognitao::machine::fsm::State::ptr& state,
		const CompiledMachines& machines,
		CompilationObjectsCollector& collector
		)
{

	cognitao::machine::parallel::MachineList m_machines;
	BOOST_FOREACH( const cognitao::io::parser::core::Node& n, node.nodes() )
	{
		if( n.name() != "machine" ) continue;
		string n_id = read_node_id(n);

		if( machines.find(n_id) == machines.end() ) throw CompilerError()<<"Machine wring name. "<<n.str()<<IN_FILE_LOCATION;

		m_machines.add( (cognitao::machine::MachineDefinitionRef&)(*(machines.at(n_id))) );
	}

	CompiledMachine _m( new cognitao::machine::MachineDefinitionRef(
			new cognitao::machine::parallel::ParallelDefinition(
					"all",
					processor,
					m_machines,
					stop_condition
			)
	) );
	collector.machines_definitions.push_back( _m );


	string state_id = read_node_id(node);
	state = cognitao::machine::fsm::State::ptr ( new cognitao::machine::fsm::StateMachine(state_id, *_m) );
}

int FsmBuilder::detect_type_of_state( const cognitao::io::parser::core::Node& node )
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

void FsmBuilder::build_state( const cognitao::io::parser::core::Node& state_node, StatesTable& states, const CompiledMachines& machines )
{
	string state_id;
	try{
		state_id = read_node_id( state_node );
	}catch(const CompilerError& err ){ throw err<<"State name is wrong. "<<state_node.str()<<IN_FILE_LOCATION; }

	add_transitions( state_node, states, states[state_id] );
}

void FsmBuilder::init_state(
		const cognitao::io::parser::core::Node& state_node,
		StatesTable& states, const CompiledMachines& machines,
		CompilationObjectsCollector& collector
		)
{
	string state_id;
	try{
		state_id = read_node_id( state_node );
	}catch(const CompilerError& err ){ throw err<<"State name is wrong. "<<state_node.str()<<IN_FILE_LOCATION; }

	int state_type = detect_type_of_state( state_node );

	switch( state_type )
	{
		case 0: build_simple_state( state_node, states, states[state_id] ); break;
		case 1: build_machine_state( state_node, states, states[state_id], machines ); break;
		case 2: build_parallel_state( state_node, states, states[state_id], machines, collector ); break;
	};
	cout<<"   "<<"state "<<state_id<<" has "<<(states[state_id]?"been defined":"not been defined")<<endl;
}

void FsmBuilder::read_states( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::States& states_results )
{
	BOOST_FOREACH( const cognitao::io::compiler::Node& sub_node, node.nodes() )
	{
		cognitao::machine::fsm::State::ptr state;
		if( sub_node.name() == "state" )
		{
			read_state( sub_node, states, state );

			states_results += state;
		}
	}
}

string FsmBuilder::read_node_data( const cognitao::io::parser::core::Node& node )
{
	string name = node.data();
	boost::trim(name);
	if( name.empty() ) throw cognitao::io::compiler::CompilerError()<<"Start state name is wrong. "<<node.str()<<IN_FILE_LOCATION;
	return name;
}
string FsmBuilder::read_node_id( const cognitao::io::parser::core::Node& node )
{
	string name = node["id"];
	boost::trim(name);
	if( name.empty() ) throw cognitao::io::compiler::CompilerError()<<"Start state name is wrong. "<<node.str()<<IN_FILE_LOCATION;
	return name;
}

void FsmBuilder::read_state( const cognitao::io::parser::core::Node& node, StatesTable& states, cognitao::machine::fsm::State::ptr& state_results )
{
	string state_name;
	try{
		state_name = read_node_data( node );
	}catch(const CompilerError& err ){ throw err<<"Start state name is wrong. "<<node.str()<<IN_FILE_LOCATION; }

	if( states.find( state_name ) == states.end() )
		throw cognitao::io::compiler::CompilerError()<<"Start state name ["<<state_name<<"] is wrong, because such state has not been defined "<<node.str()<<IN_FILE_LOCATION;

	cognitao::machine::fsm::State::ptr stateRef = states.at( state_name );
	if( not stateRef ) throw cognitao::io::compiler::CompilerError()<<"state "<<state_name<<" does not defined."<<node.str()<<IN_FILE_LOCATION;

	state_results = stateRef;
}

void FsmBuilder::build_machine( const string& machine_name, const cognitao::io::parser::core::Node& fsm_node, StatesTable& states, cognitao::machine::MachineDefinitionRef& machine )
{

	cognitao::machine::fsm::States all_states;
	BOOST_FOREACH( StatesTable::value_type& v, states )
	{
		all_states += v.second;
	}

	cognitao::machine::fsm::States success_states;
	cognitao::machine::fsm::States fail_states;

	cognitao::machine::fsm::State::ptr start_state;

	BOOST_FOREACH( const cognitao::io::compiler::Node& sub_node, fsm_node.nodes() )
	{
		if( sub_node.name() == "success" )	read_states( sub_node, states, success_states );
		if( sub_node.name() == "fail" ) 	read_states( sub_node, states, success_states );
		if( sub_node.name() == "start" ) 	read_state ( sub_node, states, start_state);
	}

	if( not start_state ) throw cognitao::io::compiler::CompilerError()<<"Start state name is not defined. "<<fsm_node.str()<<IN_FILE_LOCATION;

	machine = cognitao::machine::MachineDefinitionRef(
			new cognitao::machine::fsm::FsmDefinition(
					machine_name, processor,
					all_states, *start_state,
					success_states , fail_states
			)
	);
}

FsmBuilder::FsmBuilder( cognitao::machine::EventProcessor& processor )
: processor( processor )
{

}

string FsmBuilder::machine_type()const
{
	return "fsm";
}

void FsmBuilder::build_machine(
		const cognitao::io::parser::core::Node& node,
		const CompiledMachines& machines ,
		CompilationObjectsCollector& collector,
		CompiledMachine& machine
		)
{
	validate_machine_node( node );
	cout<<" Build FSM machine from token node "<< node["id"] <<" "<< node.str() <<endl;

	StatesTable states;
	BOOST_FOREACH( const cognitao::io::compiler::Node& machine_sub_node , node.nodes() )
	{
		if( machine_sub_node.name() == "states" )
		{
			BOOST_FOREACH( const cognitao::io::compiler::Node& states_node , machine_sub_node.nodes() )
			{
				cout<<"State "<<states_node.name()<<" "<<states_node["id"]<<" for init"<<endl;
				init_state( states_node, states, machines, collector );
			}
			BOOST_FOREACH( const cognitao::io::compiler::Node& states_node , machine_sub_node.nodes() )
			{
				cout<<"State "<<states_node.name()<<" "<<states_node["id"]<<" for build"<<endl;
				build_state( states_node, states, machines);
			}
		}else if( machine_sub_node.name() == "fsm" )
		{
			cout<<"Fsm initialization information: "<<machine_sub_node.name()<<""<<endl;
			build_machine( node["id"], machine_sub_node, states, *machine );


		}
	}

	boost::shared_ptr<FsmCompilationData> fcd( new FsmCompilationData );
	BOOST_FOREACH( StatesTable::value_type& v, states )
	{
		fcd->states.push_back( v.second );
	}
	collector.compiled_data.push_back(fcd);

}

}
}
}
}



