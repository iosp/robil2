/*
 * MachineInstance.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */


#include <cognitao/machine/fsm/Fsm.h>

namespace cognitao {
namespace machine {
namespace fsm{


//------------------------ INSTANTIATION ----------------------------------------------------------------------

FsmInstance::FsmInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context, State& state )
	: MachineInstance(def_machine,super_context + state.name(),processor, def_machine.interface())
	, _fsm_state(state)
	, _super_context(super_context)
{

}
FsmInstance::FsmInstance( const FsmInstance& inst, State& state )
	: MachineInstance(inst._def_machine, inst._super_context + state.name(), inst._processor, inst._interface, inst._machines )
	, _fsm_state(state)
	, _super_context(inst._super_context)
{
}

FsmInstance::~FsmInstance()
{
}



void FsmInstance::state( Graph& graph , const Context& context, set<void*>& visited)const
{
	throw NOT_IMPLEMENTED();

}

FsmInstance::FsmMachine FsmInstance::process_state( const Event& event, bool& transition_found)const
{
	return _fsm_state.process(event, cast(THIS), transition_found);
}
MachineInstance* FsmInstance::clone()const
{
	return new FsmInstance(*this, _fsm_state);
}

string FsmInstance::name()const
{
	stringstream s; s<<_def_machine.name() << "/" << _fsm_state.name();
	return s.str();
}

FsmInstance::FsmMachine FsmInstance::on_state_start()const
{
	_processor.send( _fsm_state._interface.on_enter.replace_context(context()) );
	return _fsm_state.on_start( FsmInstance::cast(THIS) );
}
FsmInstance::FsmMachine FsmInstance::on_state_abort()const
{
	FsmMachine m = _fsm_state.on_stop( FsmInstance::cast(THIS) );
	_processor.send( _fsm_state._interface.on_exit.replace_context(context()) );
	return m;
}
FsmInstance::FsmMachine FsmInstance::on_state_success()const
{
	FsmMachine m = _fsm_state.on_stop( FsmInstance::cast(THIS) );
	_processor.send( _fsm_state._interface.on_exit.replace_context(context()) );
	return m;
}
FsmInstance::FsmMachine FsmInstance::on_state_fail()const
{
	FsmMachine m = _fsm_state.on_stop( FsmInstance::cast(THIS) );
	_processor.send( _fsm_state._interface.on_exit.replace_context(context()) );
	_processor.send( _interface.fail_report.replace_context(context()) );
	return m;
}
FsmInstance::FsmMachine FsmInstance::on_state_transition( const Event& event, const State& state )const
{
	Event event_name = State::EventsInterface::state_report_transition(_fsm_state.name(),event.name(),state.name());
	FsmMachine m = FsmInstance::cast(THIS);
	_processor.send( Event(event_name,context()) );
	return m;
}

FsmInstance::FsmMachine FsmInstance::on_machine_start()const
{
	_processor.send( _interface.start_report.replace_context(super_context()) );
	return FsmInstance::cast(THIS);
}
FsmInstance::FsmMachine FsmInstance::on_machine_abort()const
{
	_processor.send( _interface.interrupt_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );
	return FsmInstance::FsmMachine();
}
FsmInstance::FsmMachine FsmInstance::on_machine_success(Events& private_events)const
{
	Events es = _interface.private_success_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	_processor.send( _interface.success_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );

	return FsmInstance::FsmMachine();
}
FsmInstance::FsmMachine FsmInstance::on_machine_fail(Events& private_events)const
{
	Events es = _interface.private_fail_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	_processor.send( _interface.fail_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );

	return FsmInstance::FsmMachine();
}

//------------------------ DEFINITION ----------------------------------------------------------------------


FsmDefinition::FsmDefinition(
		const Name& name,
		EventProcessor& processor,
		States states,
		State& s0,
		States succ,
		States fail
)
	: MachineDefinition( name, processor)
	, _states(states)
	, _start_state(&s0)
	, _success_states(succ)
	, _failure_states(fail)
{

}

FsmDefinition::~FsmDefinition()
{


}

Machine FsmDefinition::start_instance(const Context& caller_context, Events& private_events)const
{
	if( not _start_state ) throw Exception_MachineIsNotInitilized();

	Context context = caller_context + name();
	FsmInstance::FsmMachine inst( new FsmInstance( self(), _processor, context, *_start_state) );
	inst = inst->on_machine_start();
	inst = inst->on_state_start();
	if( is_state_successed(inst) )
	{
		inst = inst->on_state_abort();
		inst = inst->on_machine_success(private_events);
		return inst;
	}
	if( is_state_failure(inst) )
	{
		inst = inst->on_state_abort();
		inst = inst->on_machine_fail(private_events);
		return inst;
	}
	return inst;
}

bool FsmDefinition::is_event_for_interrupt( const Event& event , FsmInstance::FsmMachine& inst )const
{
	Events aborted = inst->interface().interrupt_command.matches(event, inst->super_context());
	if( aborted.empty() == false )
	{
		foreach( const Event& e, aborted.events() ) _processor.on_match(e, event);
		inst = inst->on_state_abort();
		inst = inst->on_machine_abort();
		return true;
	}
	return false;
}
bool FsmDefinition::is_event_for_success_stop( const Event& event , FsmInstance::FsmMachine& inst, Events& private_events )const
{
	Events successed = inst->interface().success_command.matches(event, inst->super_context());
	if( successed.empty() == false )
	{
		foreach( const Event& e, successed.events() ) _processor.on_match(e, event);
		inst = inst->on_state_success();
		inst = inst->on_machine_success(private_events);
		return true;
	}
	return false;
}
bool FsmDefinition::is_event_for_failure_stop( const Event& event , FsmInstance::FsmMachine& inst, Events& private_events )const
{
	Events failed = inst->interface().fail_command.matches(event, inst->super_context());
	if( failed.empty() == false )
	{
		foreach( const Event& e, failed.events() ) _processor.on_match(e, event);
		inst = inst->on_state_fail();
		inst = inst->on_machine_fail(private_events);
		return true;
	}
	return false;
}

bool FsmDefinition::is_state_successed( FsmInstance::FsmMachine& inst )const
{
	if( contains_state( _success_states, inst->_fsm_state ) == false ) return false;
	return true;
}
bool FsmDefinition::is_state_failure( FsmInstance::FsmMachine& inst )const
{
	if( contains_state( _failure_states, inst->_fsm_state ) == false ) return false;
	return true;
}


Machine FsmDefinition::process( const Event& event , Machine instance, Events& private_events )const
{
	FsmInstance::FsmMachine inst = FsmInstance::cast(instance);

	if( is_event_for_interrupt(event, inst) ) return inst;
	if( is_event_for_success_stop(event, inst, private_events) ) return inst;
	if( is_event_for_failure_stop(event, inst, private_events) ) return inst;

	bool transition_found = false;
	FsmInstance::FsmMachine new_state = inst->process_state(event, transition_found);

	/* NOTE:
	 *   don't check if new_task == inst, because it is can be false even if task did not changed.
	 *   It can be different instance, because internal machines changes.
	 */
	if( transition_found )
	{
		// stop previous task
		inst = inst->on_state_success();

		// start new task
		inst = new_state->on_state_start();

		// check if new task is a Final Task (success or failure)
		if( is_state_successed(inst) )
		{
			inst = inst->on_state_abort();
			inst = inst->on_machine_success(private_events);
			return inst;
		}
		if( is_state_failure(inst) )
		{
			inst = inst->on_state_abort();
			inst = inst->on_machine_fail(private_events);
			return inst;
		}
	}else{
		// no transition => no changes in task.
		inst = new_state;
	}

	return inst;
}

void FsmDefinition::state( Graph& graph , const Context& context, set<void*>& visited)const
{
	//throw NOT_IMPLEMENTED();
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Context lcontext = context+name();
	graph::Node* s = graph.node( lcontext.str() );
	s->property("machine") = "fsm";

	graph::Node* n = graph.node( (lcontext + _start_state->name()).str() );
	n->property("started")="true";
	graph.edge( s, n, lcontext.str()+":"+"start", "start" );

	foreach( const States::value_type& v, _states )
	{
		cout<<"E "<<v->name()<<endl;
		graph::Node* n = graph.node( (lcontext + v->name()).str() );
		graph.edge( s, n, lcontext.str()+":"+"contains_"+v->name(), "contains" );
		v->state( graph, lcontext, visited);
	}

}


//-------------------------------------- STATES ------------------------------------------

State::State( const Name& name )
	: _name(name)
{

}

StateMachine::StateMachine( const State::Name& name , MachineDefinition& machine_def )
	: State(name)
	, _def_machine(&machine_def)
	, _def_machine_ref(0)
{

}
StateMachine::StateMachine( const State::Name& name , MachineDefinitionRef& machine_def )
	: State(name)
	, _def_machine(0)
	, _def_machine_ref(&machine_def)
{

}


void State::state( Graph& g, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Node* n = g.node( (context + name()).str() );
	foreach( const Condition::ptr& c, _conditions )
	{
		c->state( g,n, context,visited );
	}

}


void State::add_condition( Condition::ptr cond )
{
	_conditions.push_back(cond);
}


bool Transition::test( const Event& event , const Context& context, EventProcessor& processor  )const
{
	Events matched = trigers.matches( event, context );
	if( matched.empty() == false )
	{
		processor.on_match( matched.events().front() , event );
		return true;
	}
	return false;
}

void Transition::state( Graph& g, Node* source, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Node* t = g.node( (context+target.name()).str() );
	g.edge( source, t, source->id()+":transition_to_"+target.name(), "fsm_transition" );
	//target.state(g, context, visited);
}

FsmInstance::FsmMachine State::on_start( FsmInstance::FsmMachine inst )const
{
	if( inst->sub_machines().size() > 0 )
	{
		/*
		 * if current inst has a sub machines, it has got it from previous state.
		 * Actually this sub machines have been stopped and inst contains just a pointers (copied during clone operation)
		 * In FSM when parent is dead, children are dead too, so, this sub machines are not in life. remove it without
		 * any additional cleanups.
		 */
		FsmInstance* fm = (FsmInstance*) inst->clone();
		FsmInstance::FsmMachine nm(fm);
		fm->clear_machines();
		return nm;
	}
	return inst;
}

FsmInstance::FsmMachine State::on_stop( FsmInstance::FsmMachine inst )const
{
	return inst;
}

FsmInstance::FsmMachine State::process( const Event& event , FsmInstance::FsmMachine inst, bool& transition_found )const
{
	const Context& context = inst->context();
	foreach( const Condition::ptr& cond, _conditions )
	{
		if( cond->test( event , context, inst->processor() ) )
		{
			transition_found = true;
			inst->on_state_transition( event, cond->target );
			foreach( const Event& out_ev, cond->outputs.events() )
			{
				inst->processor().send( Event( out_ev, inst->context()) );
			}
			return FsmInstance::FsmMachine( new FsmInstance( *inst, cond->target ) );
		}
	}
	return inst;
}

//--------------------- StateMachine -----------------------------

void  StateMachine::state( Graph& g, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Context lcontext = context + name();
	Node* n = g.node( lcontext.str() );
	n->property("type")="machine";

	foreach( const Condition::ptr& c, _conditions )
	{
		c->state( g,n, context,visited );
	}

	g.edge( n, g.node( (lcontext + def_machine().name()).str() ), lcontext.str()+":sub_machine", "sub_machine");

	def_machine().state(g, lcontext, visited);
}


MachineDefinition& StateMachine::def_machine()const
{

	if( _def_machine ) return *_def_machine;
	if( _def_machine_ref and _def_machine_ref->machine)
	{
		return *(_def_machine_ref->machine);
	}
	throw Exception_MachineIsNotInitilized();
}



FsmInstance::FsmMachine StateMachine::on_start( FsmInstance::FsmMachine inst )const
{

	Events sub_machine_private_events;
	Machine sub_machine = def_machine().start_instance(inst->context(), sub_machine_private_events);

	FsmInstance* fm = (FsmInstance*) inst->clone();
	FsmInstance::FsmMachine nm(fm);
	fm->clear_machines();
	if(sub_machine)
		fm->add_machine( sub_machine );

	return nm;
}

FsmInstance::FsmMachine StateMachine::on_stop( FsmInstance::FsmMachine inst )const
{
	inst->interrupt_submachines();
	FsmInstance* fm = (FsmInstance*) inst->clone();
	FsmInstance::FsmMachine m( fm );
	fm->clear_machines();
	return m;
}



}
}
}

