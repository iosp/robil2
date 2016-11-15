/*
 * MachineInstance.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */


#include <cognitao/machine/parallel/Parallel.h>

namespace cognitao {
namespace machine {
namespace parallel{


//------------------------ INSTANTIATION ----------------------------------------------------------------------

ParallelInstance::ParallelInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context)
	: MachineInstance(def_machine,super_context,processor, def_machine.interface())
	, _super_context(super_context)
{

}
ParallelInstance::ParallelInstance( const ParallelInstance& inst )
	: MachineInstance(inst._def_machine, inst._super_context, inst._processor, inst._interface, inst._machines )
	, _super_context(inst._super_context)
	, _statistic(inst._statistic)
{
}

ParallelInstance::~ParallelInstance()
{
}


void ParallelInstance::state(  Graph& graph , const Context& context, set<void*>& visited)const
{
	throw NOT_IMPLEMENTED();

}

MachineInstance* ParallelInstance::clone()const
{
	return new ParallelInstance(*this );
}

string ParallelInstance::name()const
{
	throw NOT_IMPLEMENTED();
	return "";
}

ParallelInstance::PrlMachine ParallelInstance::on_machine_start()const
{
	//std::cout << "[MACHINE] ParallelInstance::on_machine_start :"<<inst_id <<endl;

	_processor.send( _interface.start_report.replace_context(super_context()) );
	return ParallelInstance::cast(THIS);
}
ParallelInstance::PrlMachine ParallelInstance::on_machine_abort()const
{
	//std::cout << "[MACHINE] ParallelInstance::on_machine_abort :"<<inst_id <<endl;

	interrupt_submachines();
	ParallelInstance::EditablePrlMachine without_subs( (ParallelInstance*) clone() );
	without_subs->clear_machines();

	_processor.send( _interface.interrupt_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );
	return ParallelInstance::PrlMachine();
}
ParallelInstance::PrlMachine ParallelInstance::on_machine_success(Events& private_events)const
{
	//std::cout << "[MACHINE] ParallelInstance::on_machine_success :"<<inst_id <<endl;

	Events es = _interface.private_success_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	interrupt_submachines();
	ParallelInstance::EditablePrlMachine without_subs( (ParallelInstance*) clone() );
	without_subs->clear_machines();

	_processor.send( _interface.success_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );
	return ParallelInstance::PrlMachine();
}
ParallelInstance::PrlMachine ParallelInstance::on_machine_fail(Events& private_events)const
{
	//std::cout << "[MACHINE] ParallelInstance::on_machine_fail  :"<<inst_id <<endl;

	Events es = _interface.private_fail_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	interrupt_submachines();
	ParallelInstance::EditablePrlMachine without_subs( (ParallelInstance*) clone() );
	without_subs->clear_machines();

	_processor.send( _interface.fail_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );
	return ParallelInstance::PrlMachine();
}

//------------------------ DEFINITION ----------------------------------------------------------------------


ParallelDefinition::ParallelDefinition(
		const Name& name,
		EventProcessor& processor,
		const MachineList& sub_machines
)
	: MachineDefinition( name, processor)
	, _def_machines(sub_machines)
{

}

ParallelDefinition::~ParallelDefinition()
{


}

Machine ParallelDefinition::start_instance(const Context& caller_context, Events& private_events)const
{

	Context context = caller_context + name();
	Machine result;

	ParallelInstance::EditablePrlMachine parallel( new ParallelInstance( self(), _processor, context ) );

	Events prv_events;
	foreach( MachineRef def, _def_machines.machines )
	{
		Machine m = def.def_machine().start_instance( context, prv_events );
		if(m) parallel->add_machine( m );
	}

	parallel->_statistic.total_count = _def_machines.machines.size();
	if( not parallel ) return parallel;

	result = parallel->on_machine_start();

	while( prv_events and result ){
		Event e = prv_events.pop();
		result = result->process( e, private_events );
	}

	return result;
}

bool ParallelDefinition::is_event_for_interrupt( const Event& event , ParallelInstance::PrlMachine& inst )const
{
	Events aborted = inst->interface().interrupt_command.matches(event, inst->super_context());
	if( aborted.empty() == false )
	{
		foreach( const Event& e, aborted.events() ) _processor.on_match(e, event);
		inst = inst->on_machine_abort();
		return true;
	}
	return false;
}
bool ParallelDefinition::is_event_for_success_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const
{

	Event child_stop = Event(EventsInterface::machine_command_success(), inst->super_context());
	if( event.matches( child_stop ) )
	{
		if( _stop_condition) _processor.on_match( child_stop, event );

		ParallelInstance::EditablePrlMachine st_updated((ParallelInstance*)inst->clone());
		st_updated->_statistic.successed_count++;
		inst = st_updated;
		if( _stop_condition) return on_child_stop( event, inst, private_events );
	}

	Events successed = inst->interface().success_command.matches(event, inst->super_context());
	if( successed.empty() == false )
	{
		foreach( const Event& e, successed.events() ) _processor.on_match(e, event);
		inst = inst->on_machine_success(private_events);
		return true;
	}
	return false;
}

bool ParallelDefinition::is_event_for_failure_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const
{

	Event child_stop = Event(EventsInterface::machine_command_fail(), inst->super_context());
	if( event.matches( child_stop ) )
	{
		if( _stop_condition) _processor.on_match( child_stop, event );

		ParallelInstance::EditablePrlMachine st_updated((ParallelInstance*)inst->clone());
		st_updated->_statistic.failures_count++;
		inst = st_updated;
		if( _stop_condition) return on_child_stop( event, inst, private_events );
	}

	Events failed = inst->interface().fail_command.matches(event, inst->super_context());
	if( failed.empty() == false )
	{
		foreach( const Event& e, failed.events() ) _processor.on_match(e, event);
		inst = inst->on_machine_fail(private_events);
		return true;
	}
	return false;
}

bool ParallelDefinition::on_child_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const
{
	int i = _stop_condition->test( inst->_statistic );
	if( i!=0 )
	{
		if( i > 0 ) inst = inst->on_machine_success(private_events);
		if( i < 0 ) inst = inst->on_machine_fail(private_events);
		return true;
	}
	return false;
}


Machine ParallelDefinition::process( const Event& event , Machine instance, Events& private_events )const
{
	ParallelInstance::PrlMachine inst = ParallelInstance::cast(instance);

	if( is_event_for_interrupt(event, inst) ) return inst;
	if( is_event_for_success_stop(event, inst, private_events ) ) return inst;
	if( is_event_for_failure_stop(event, inst, private_events ) ) return inst;

	return inst;
}

void ParallelDefinition::state(  Graph& graph , const Context& context, set<void*>& visited)const
{
	if( visited.find((void*)this) != visited.end() ) return;
	visited.insert((void*)this);

	Context lcontext = context + name();
	Node* s = graph.node( lcontext.str() );
	s->property("machine") = "parallel";

	BOOST_FOREACH( const parallel::MachineRef& ref , _def_machines.machines )
	{
		Context c = lcontext + ref.def_machine().name();
		Node* n = graph.node( c.str() );

		graph.edge( s, n, lcontext.str()+":contains_"+ref.def_machine().name(), "contains" );

		ref.def_machine().state( graph, lcontext, visited );
	}
}

//---------------------------- MACHINES -----------------------------

MachineDefinition& MachineRef::def_machine()const
{
	if( _def ) return *_def;
	if( _def_ref and _def_ref->machine ) return *(_def_ref->machine);
	throw Exception_MachineIsNotInitilized();
}


}
}
}

