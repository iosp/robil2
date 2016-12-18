
#ifndef COGNITEAO_MACHINES_SRC_ftt_DEF_H_
#define COGNITEAO_MACHINES_SRC_ftt_DEF_H_

#include "FttInstance.h"

namespace cognitao{
namespace machine{
namespace ftt{

class Condition
{
public:
	typedef boost::shared_ptr< Condition > ptr;


	Events outputs;
	Task& target;

	Condition( Events outputs, Task& target )
		: outputs(outputs)
		, target(target)
	{

	}

	virtual
	~Condition(){}

	virtual
	void state( Graph& graph, Node* source, const Context& context, set<void*>& visited )const=0;

	virtual
	bool test( const Event& event , const Context& context, EventProcessor& processor )const=0;

};

class Reaction: public Condition
{
public:
	Events trigers;

	Reaction( Events trigers, Events outputs, Task& target )
		: Condition(outputs, target), trigers(trigers)
	{

	}

	bool test( const Event& event , const Context& context, EventProcessor& processor )const;

	void state( Graph& graph, Node* source, const Context& context, set<void*>& visited )const;
};

typedef std::vector<Condition::ptr> Conditions;



class Task
{
public:

	struct EventsInterface
	{
		static std::string task_report_channel(){ return "task_report"; }

		Events on_enter;
		Events on_exit;

		static const Event task_report_enter(bool out=true){ return Event( task_report_channel() + (out?"!":"?") + "enter" ); }
		static const Event task_report_exit (bool out=true){ return Event( task_report_channel() + (out?"!":"?") + "exit"  ); }
		static const Event task_report_transition (string source, string event, string target, bool out=true)
												       { return Event( task_report_channel() + (out?"!":"?") + "transition_"+source+"_"+event+"_"+target ); }

		EventsInterface()
		{
			on_enter.add( Event( task_report_enter() ) );
			on_exit.add( Event( task_report_exit() ) );
		}
	};


	typedef boost::shared_ptr< Task > ptr;
	typedef std::string Name;
	typedef std::string Arguments;
	typedef std::list<Name> Aliases;

	Name _name;
	Conditions _conditions;
	EventsInterface _interface;
	Arguments _arguments;
	Aliases _aliases;

#define ALIASE_AS_SUB_ELEMENT

	Name name()const{ return _name; }
	Arguments arguments()const{ return _arguments; }

	Task( const Name& name , const Arguments& arg );
	Task( const Name& name , const Name& behavior, const Arguments& arg );

	virtual
	~Task(){}

	virtual
	void  state( Graph& graph, const Context& context, set<void*>& visited )const;

	void add_condition( Condition::ptr cond );

	virtual
	FttInstance::FttMachine process( const Event& event , FttInstance::FttMachine inst, bool& transition_found )const;

	virtual
	FttInstance::FttMachine on_start( FttInstance::FttMachine inst )const;

	virtual
	FttInstance::FttMachine on_stop( FttInstance::FttMachine inst )const;

	Events setup_enter_event( const Context& context )const;
	Events setup_exit_event( const Context& context )const;

};

class TaskMachine: public Task
{
public:
	MachineDefinition* _def_machine;
	MachineDefinitionRef* _def_machine_ref;

	TaskMachine( const Task::Name& name, const Arguments& arg, MachineDefinition& machine_def );
	TaskMachine( const Task::Name& name, const Arguments& arg, MachineDefinitionRef& machine_def );
	TaskMachine( const Task::Name& name , const Name& behavior, const Arguments& arg, MachineDefinition& machine_def );
	TaskMachine( const Task::Name& name , const Name& behavior, const Arguments& arg, MachineDefinitionRef& machine_def );

	virtual
	~TaskMachine()
	{
		_def_machine = 0;
		_def_machine_ref = 0;
	}

	MachineDefinition& def_machine()const;

	FttInstance::FttMachine on_start( FttInstance::FttMachine inst )const;
	FttInstance::FttMachine on_stop( FttInstance::FttMachine inst )const;

	virtual
	void  state( Graph& graph, const Context& context, set<void*>& visited )const;

};

typedef std::vector<Task::ptr> Tasks;

inline
bool contains_task(const Tasks& s, const Task::ptr& p)
{
	foreach( const Task::ptr& t, s ) if( p.get() == t.get() ) return true;
	return false;
}
inline
bool contains_task(const Tasks& s, const Task *const p)
{
	foreach( const Task::ptr& t, s ) if( p == t.get() ) return true;
	return false;
}
inline
bool contains_task(const Tasks& s, const Task & p)
{
	foreach( const Task::ptr& t, s ) if( &p == t.get() ) return true;
	return false;
}

inline
Tasks& operator+=( Tasks& ss, Task::ptr s )
{
	ss.push_back(s);
	return ss;
}
inline
Tasks& operator+=( Tasks& s1, const Tasks& s2 )
{
	s1.insert(s1.end(), s2.begin(), s2.end());
	return s1;
}
inline
Tasks operator+( Task::ptr s1, Task::ptr s2 )
{
	Tasks s;
	s += s1;
	s += s2;
	return s;
}
inline
Tasks operator+( const Tasks& s1, Task::ptr s2 )
{
	Tasks s(s1);
	s += s2;
	return s;
}

class FttDefinition: public MachineDefinition
{
	friend class FttInstance;
public:

private:

	Tasks _tasks;
	Task* _start_task;

	Tasks _success_tasks;
	Tasks _failure_tasks;

public:

	FttDefinition(
			const Name& name,
			EventProcessor& processor,
			Tasks tasks,
			Task& s0,
			Tasks succ = Tasks(),
			Tasks fail = Tasks()
	);

	virtual
	~FttDefinition();

	virtual
	Machine start_instance(const Context& context, Events& private_events)const;

	virtual
	void state( Graph& graph , const Context& context, set<void*>& visited)const;


protected:

	virtual
	Machine process( const Event& event , Machine instance, Events& private_events )const;


	bool is_event_for_interrupt( const Event& event , FttInstance::FttMachine& inst )const;
	bool is_event_for_success_stop( const Event& event , FttInstance::FttMachine& inst, Events& private_events )const;
	bool is_event_for_failure_stop( const Event& event , FttInstance::FttMachine& inst, Events& private_events )const;

	bool is_task_successed( FttInstance::FttMachine& inst )const;
	bool is_task_failure( FttInstance::FttMachine& inst )const;


};


}
}
}


#endif /* COGNITEAO_MACHINES_SRC_FSM_DEF_H_ */
