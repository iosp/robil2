/*
 * MachineInstance.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */


#include <cognitao/machine/ftt/Ftt.h>

namespace cognitao {
namespace machine {
namespace ftt{


//------------------------ INSTANTIATION ----------------------------------------------------------------------

FttInstance::FttInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context, Task& task )
	: MachineInstance(def_machine,super_context + task.name(),processor, def_machine.interface())
	, _ftt_task(task)
	, _super_context(super_context)
{

}
FttInstance::FttInstance( const FttInstance& inst, Task& task )
	: MachineInstance(inst._def_machine, inst._super_context + task.name(), inst._processor, inst._interface, inst._machines )
	, _ftt_task(task)
	, _super_context(inst._super_context)
{
}

FttInstance::~FttInstance()
{
}



void FttInstance::state( Graph& graph , const Context& context, set<void*>& visited)const
{
	throw NOT_IMPLEMENTED();

}

FttInstance::FttMachine FttInstance::process_task( const Event& event, bool& transition_found)const
{
	return _ftt_task.process(event, cast(THIS), transition_found);
}
MachineInstance* FttInstance::clone()const
{
	return new FttInstance(*this, _ftt_task);
}

string FttInstance::name()const
{
	stringstream s; s<<_def_machine.name() << "/" << _ftt_task.name();
	return s.str();
}

FttInstance::FttMachine FttInstance::on_task_start()const
{
	_processor.send( _ftt_task.setup_enter_event(context()) );
	return _ftt_task.on_start( FttInstance::cast(THIS) );
}
FttInstance::FttMachine FttInstance::on_task_abort()const
{
	FttMachine m = _ftt_task.on_stop( FttInstance::cast(THIS) );
	_processor.send( _ftt_task.setup_exit_event(context()) );
	return m;
}
FttInstance::FttMachine FttInstance::on_task_success()const
{
	FttMachine m = _ftt_task.on_stop( FttInstance::cast(THIS) );
	_processor.send( _ftt_task.setup_exit_event(context()) );
	return m;
}
FttInstance::FttMachine FttInstance::on_task_fail()const
{
	FttMachine m = _ftt_task.on_stop( FttInstance::cast(THIS) );
	_processor.send( _ftt_task.setup_exit_event(context()) );
	_processor.send( _interface.fail_report.replace_context(context()) );
	return m;
}
FttInstance::FttMachine FttInstance::on_task_transition( const Event& event, const Task& task )const
{
	Event event_name = Task::EventsInterface::task_report_transition(_ftt_task.name(),event.name(),task.name());
	Events output_events( event_name );
	FttMachine m = FttInstance::cast(THIS);
	_processor.send( output_events.replace_context(context()) );
	return m;
}

FttInstance::FttMachine FttInstance::on_machine_start()const
{
	_processor.send( _interface.start_report.replace_context(super_context()) );
	return FttInstance::cast(THIS);
}
FttInstance::FttMachine FttInstance::on_machine_abort()const
{
	_processor.send( _interface.interrupt_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );
	return FttInstance::FttMachine();
}
FttInstance::FttMachine FttInstance::on_machine_success(Events& private_events)const
{
	Events es = _interface.private_success_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	_processor.send( _interface.success_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );

	return FttInstance::FttMachine();
}
FttInstance::FttMachine FttInstance::on_machine_fail(Events& private_events)const
{
	Events es = _interface.private_fail_report.replace_context(super_context());
	_processor.on_private( es );
	private_events += es;

	_processor.send( _interface.fail_report.replace_context(super_context()) );
	_processor.send( _interface.stop_report.replace_context(super_context()) );

	return FttInstance::FttMachine();
}

//------------------------ DEFINITION ----------------------------------------------------------------------


FttDefinition::FttDefinition(
		const Name& name,
		EventProcessor& processor,
		Tasks tasks,
		Task& s0,
		Tasks succ,
		Tasks fail
)
	: MachineDefinition( name, processor)
	, _tasks(tasks)
	, _start_task(&s0)
	, _success_tasks(succ)
	, _failure_tasks(fail)
{

}

FttDefinition::~FttDefinition()
{


}

Machine FttDefinition::start_instance(const Context& caller_context, Events& private_events)const
{
	if( not _start_task ) throw Exception_MachineIsNotInitilized();

	Context context = caller_context + name();
	FttInstance::FttMachine inst( new FttInstance( self(), _processor, context, *_start_task) );
	inst = inst->on_machine_start();
	inst = inst->on_task_start();
	std::cout<<"[MACHINE EXECUTION] machine "<<name()<<" ???? "<<inst->_ftt_task.name()<<"   "<<_success_tasks.size()<<endl;
	BOOST_FOREACH( Task::ptr t, _success_tasks)
	{
		std::cout<<"[MACHINE EXECUTION] ... success tasks = "<<t->name()<<endl;
	}
	if( is_task_successed(inst) )
	{
		std::cout<<"[MACHINE EXECUTION] machine "<<name()<<" is success"<<endl;
		inst = inst->on_task_abort();
		inst = inst->on_machine_success(private_events);
		return inst;
	}
	if( is_task_failure(inst) )
	{
		inst = inst->on_task_abort();
		inst = inst->on_machine_fail(private_events);
		return inst;
	}
	return inst;
}

bool FttDefinition::is_event_for_interrupt( const Event& event , FttInstance::FttMachine& inst )const
{
	Events aborted = inst->interface().interrupt_command.matches(event, inst->super_context());
	if( aborted.empty() == false )
	{
		foreach( const Event& e, aborted.events() ) _processor.on_match(e, event);
		inst = inst->on_task_abort();
		inst = inst->on_machine_abort();
		return true;
	}
	return false;
}
bool FttDefinition::is_event_for_success_stop( const Event& event , FttInstance::FttMachine& inst, Events& private_events )const
{
	Events successed = inst->interface().success_command.matches(event, inst->super_context());
	if( successed.empty() == false )
	{
		foreach( const Event& e, successed.events() ) _processor.on_match(e, event);
		inst = inst->on_task_success();
		inst = inst->on_machine_success(private_events);
		return true;
	}
	return false;
}
bool FttDefinition::is_event_for_failure_stop( const Event& event , FttInstance::FttMachine& inst, Events& private_events )const
{
	Events failed = inst->interface().fail_command.matches(event, inst->super_context());
	if( failed.empty() == false )
	{
		foreach( const Event& e, failed.events() ) _processor.on_match(e, event);
		inst = inst->on_task_fail();
		inst = inst->on_machine_fail(private_events);
		return true;
	}
	return false;
}

bool FttDefinition::is_task_successed( FttInstance::FttMachine& inst )const
{
	if( contains_task( _success_tasks, inst->_ftt_task ) == false ) return false;
	return true;
}
bool FttDefinition::is_task_failure( FttInstance::FttMachine& inst )const
{
	if( contains_task( _failure_tasks, inst->_ftt_task ) == false ) return false;
	return true;
}


Machine FttDefinition::process( const Event& event , Machine instance, Events& private_events )const
{
	FttInstance::FttMachine inst = FttInstance::cast((FttInstance::Machine)instance);

	if( is_event_for_interrupt(event, inst) ) return inst;
	if( is_event_for_success_stop(event, inst, private_events) ) return inst;
	if( is_event_for_failure_stop(event, inst, private_events) ) return inst;

	bool transition_found = false;
	FttInstance::FttMachine new_task = inst->process_task(event, transition_found);

	/* NOTE:
	 *   don't check if new_task == inst, because it is can be false even if task did not changed.
	 *   It can be different instance, because internal machines changes.
	 */
	if( transition_found ) // new task is found.
	{
		// stop previous task
		inst = inst->on_task_success();

		// start new task
		inst = new_task->on_task_start();

		// check if new task is a Final Task (success or failure)
		if( is_task_successed(inst) )
		{
			inst = inst->on_task_abort();
			inst = inst->on_machine_success(private_events);
			return inst;
		}
		if( is_task_failure(inst) )
		{
			inst = inst->on_task_abort();
			inst = inst->on_machine_fail(private_events);
			return inst;
		}

	}else{
		// no transition => no changes in task.
		inst = new_task;
	}

	return inst;
}

void FttDefinition::state( Graph& graph , const Context& context, set<void*>& visited)const
{
	//throw NOT_IMPLEMENTED();
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Context lcontext = context+name();
	graph::Node* s = graph.node( lcontext.str() );
	s->property("machine") = "ftt";

	graph::Node* n = graph.node( (lcontext + _start_task->name()).str() );
	n->property("started")="true";
	graph.edge( s, n, lcontext.str()+":"+"start", "start" );

	foreach( const Tasks::value_type& v, _tasks )
	{
		cout<<"E "<<v->name()<<endl;
		graph::Node* n = graph.node( (lcontext + v->name()).str() );
		graph.edge( s, n, lcontext.str()+":"+"contains_"+v->name(), "contains" );
		v->state( graph, lcontext, visited);
	}

}


//-------------------------------------- STATES ------------------------------------------
template<class C, class T>
void insert( C& lst, T& element )
{
	BOOST_FOREACH( typename C::value_type& e, lst ) if( element == e ) return;
	lst.push_back(element);
}


Task::Task( const Name& name, const Arguments& arg )
	: _name(name)
	, _arguments(arg)
{
#ifndef ALIASE_AS_SUB_ELEMENT
	insert(_aliases, _name);
#else
	insert(_aliases, "");
#endif
}

Task::Task( const Name& name, const Name& behavior, const Arguments& arg )
	: _name(name)
	, _arguments(arg)
{
#ifndef ALIASE_AS_SUB_ELEMENT
	insert(_aliases, _name);
	insert(_aliases, behavior);
#else
	if(behavior!=name) insert(_aliases, "");
	insert(_aliases, behavior);
#endif
}

TaskMachine::TaskMachine( const Task::Name& name , const Arguments& arg, MachineDefinition& machine_def )
	: Task(name, arg)
	, _def_machine(&machine_def)
	, _def_machine_ref(0)
{
}
TaskMachine::TaskMachine( const Task::Name& name , const Arguments& arg, MachineDefinitionRef& machine_def )
	: Task(name, arg)
	, _def_machine(0)
	, _def_machine_ref(&machine_def)
{
}

TaskMachine::TaskMachine( const Task::Name& name, const Name& behavior , const Arguments& arg, MachineDefinition& machine_def )
	: Task(name, behavior, arg)
	, _def_machine(&machine_def)
	, _def_machine_ref(0)
{
}
TaskMachine::TaskMachine( const Task::Name& name, const Name& behavior , const Arguments& arg, MachineDefinitionRef& machine_def )
	: Task(name, behavior, arg)
	, _def_machine(0)
	, _def_machine_ref(&machine_def)
{
}


void Task::state( Graph& g, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Node* n = g.node( (context + name()).str() );
	foreach( const Condition::ptr& c, _conditions )
	{
		c->state( g,n, context,visited );
	}

}

Events Task::setup_enter_event( const Context& context )const
{
	Events result;

	std::vector<Name> als(_aliases.begin(), _aliases.end());
	BOOST_FOREACH( const Name& al, als )
	{
		Context ctx = context;
#ifndef ALIASE_AS_SUB_ELEMENT
		ctx.pop();
#endif
		ctx.push(al);
		Events events = _interface.on_enter.replace_context(ctx);

		BOOST_FOREACH( const Event& e, events.events() )
		{
			Event ne(e);
			ne.parameters( arguments() );
			result.add( ne );
		}
	}

	return result;
}
Events Task::setup_exit_event( const Context& context )const
{
	Events result;

	std::vector<Name> als(_aliases.begin(), _aliases.end());
	reverse(als.begin(), als.end());
	BOOST_FOREACH( const Name& al, als )
	{
		Context ctx = context;
#ifndef ALIASE_AS_SUB_ELEMENT
		ctx.pop();
#endif
		ctx.push(al);
		Events events = _interface.on_exit.replace_context(ctx);
		result += events;
	}

	return result;
}

void Task::add_condition( Condition::ptr cond )
{
	_conditions.push_back(cond);
}


bool Reaction::test( const Event& event , const Context& context, EventProcessor& processor  )const
{
	Events matched = trigers.matches( event, context );
	if( matched.empty() == false )
	{
		processor.on_match( matched.events().front() , event );
		return true;
	}
	return false;
}

void Reaction::state( Graph& g, Node* source, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Node* t = g.node( (context+target.name()).str() );
	g.edge( source, t, source->id()+":transition_to_"+target.name(), "ftt_transition" );
	//target.task(g, context, visited);
}

FttInstance::FttMachine Task::on_start( FttInstance::FttMachine inst )const
{
	if( inst->sub_machines().size() > 0 )
	{
		/*
		 * if current inst has a sub machines, it has got it from previous task.
		 * Actually this sub machines have been stopped and inst contains just a pointers (copied during clone operation)
		 * In FTT, when parent is dead, children are dead too, so, this sub machines are not in life. remove it without
		 * any additional cleanups.
		 */
		FttInstance* fm = (FttInstance*) inst->clone();
		FttInstance::FttMachine nm(fm);
		fm->clear_machines();
		return nm;
	}
	return inst;
}

FttInstance::FttMachine Task::on_stop( FttInstance::FttMachine inst )const
{
	return inst;
}

FttInstance::FttMachine Task::process( const Event& event , FttInstance::FttMachine inst, bool& transition_found )const
{
	const Context& context = inst->context();
	foreach( const Condition::ptr& cond, _conditions )
	{
		if( cond->test( event , context, inst->processor() ) )
		{
			transition_found = true;
			inst->on_task_transition( event, cond->target );
			inst->processor().send( cond->outputs.replace_context(inst->context()) );
			return FttInstance::FttMachine( new FttInstance( *inst, cond->target ) );
		}
	}
	return inst;
}

//--------------------- TaskMachine -----------------------------

void  TaskMachine::state( Graph& g, const Context& context, set<void*>& visited )const
{
	if( visited.find( (void*)this ) != visited.end() ) return;
	visited.insert( (void*)this );

	Context lcontext = context + name();
	Node* n = g.node( lcontext.str() );
	n->property("type")="machine";

	foreach( const Condition::ptr& c, _conditions )
	{
		c->state(g,n, context,visited );
	}

	g.edge( n, g.node( (lcontext + def_machine().name()).str() ), lcontext.str()+":sub_machine", "sub_machine");

	def_machine().state(g, lcontext, visited);
}


MachineDefinition& TaskMachine::def_machine()const
{

	if( _def_machine ) return *_def_machine;
	if( _def_machine_ref and _def_machine_ref->machine)
	{
		return *(_def_machine_ref->machine);
	}
	throw Exception_MachineIsNotInitilized();
}



FttInstance::FttMachine TaskMachine::on_start( FttInstance::FttMachine inst )const
{

	Events sub_machine_private_events;
	Machine sub_machine = def_machine().start_instance(inst->context(), sub_machine_private_events);

	FttInstance* fm = (FttInstance*) inst->clone();
	FttInstance::FttMachine nm(fm);
	fm->clear_machines();
	if(sub_machine)
		fm->add_machine( (FttInstance::Machine) sub_machine );

	return nm;
}

FttInstance::FttMachine TaskMachine::on_stop( FttInstance::FttMachine inst )const
{
	inst->interrupt_submachines();
	FttInstance* fm = (FttInstance*) inst->clone();
	FttInstance::FttMachine m( fm );
	fm->clear_machines();
	return m;
}



}
}
}

