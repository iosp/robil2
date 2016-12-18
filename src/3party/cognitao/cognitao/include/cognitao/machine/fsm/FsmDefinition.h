
#ifndef COGNITEAO_MACHINES_SRC_FSM_DEF_H_
#define COGNITEAO_MACHINES_SRC_FSM_DEF_H_

#include "FsmInstance.h"

namespace cognitao{
namespace machine{
namespace fsm{

class Condition
{
public:
	typedef boost::shared_ptr< Condition > ptr;


	Events outputs;
	State& target;

	Condition( Events outputs, State& target )
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

class Transition: public Condition
{
public:
	Events trigers;

	Transition( Events trigers, Events outputs, State& target )
		: Condition(outputs, target), trigers(trigers)
	{

	}

	bool test( const Event& event , const Context& context, EventProcessor& processor )const;

	void state( Graph& graph, Node* source, const Context& context, set<void*>& visited )const;
};

typedef std::vector<Condition::ptr> Conditions;



class State
{
public:

	struct EventsInterface
	{
		static std::string state_report_channel(){ return "state_report"; }

		Events on_enter;
		Events on_exit;

		EventsInterface()
		{
			on_enter.add( Event( state_report_channel() + "!" + "enter" ) );
			on_exit.add( Event( state_report_channel()  + "!" + "exit" ) );
		}

		static const Event state_report_enter(bool out=true){ return Event( state_report_channel() + (out?"!":"?") + "enter" ); }
		static const Event state_report_exit (bool out=true){ return Event( state_report_channel() + (out?"!":"?") + "exit"  ); }
		static const Event state_report_transition (string source, string event, string target, bool out=true)
															{ return Event( state_report_channel() + (out?"!":"?") + "transition_"+source+"_"+event+"_"+target ); }
	};


	typedef boost::shared_ptr< State > ptr;
	typedef std::string Name;

	Name _name;
	Conditions _conditions;
	EventsInterface _interface;

	Name name()const{ return _name; }

	State( const Name& name );

	virtual
	~State(){}

	virtual
	void  state( Graph& graph, const Context& context, set<void*>& visited )const;

	void add_condition( Condition::ptr cond );

	virtual
	FsmInstance::FsmMachine process( const Event& event , FsmInstance::FsmMachine inst, bool& transition_found )const;

	virtual
	FsmInstance::FsmMachine on_start( FsmInstance::FsmMachine inst )const;

	virtual
	FsmInstance::FsmMachine on_stop( FsmInstance::FsmMachine inst )const;

};

class StateMachine: public State
{
public:
	MachineDefinition* _def_machine;
	MachineDefinitionRef* _def_machine_ref;

	StateMachine( const State::Name& name , MachineDefinition& machine_def );
	StateMachine( const State::Name& name , MachineDefinitionRef& machine_def );

	virtual
	~StateMachine()
	{
		_def_machine = 0;
		_def_machine_ref = 0;
	}

	MachineDefinition& def_machine()const;

	FsmInstance::FsmMachine on_start( FsmInstance::FsmMachine inst )const;
	FsmInstance::FsmMachine on_stop( FsmInstance::FsmMachine inst )const;

	virtual
	void  state( Graph& graph, const Context& context, set<void*>& visited )const;

};

typedef std::vector<State::ptr> States;

inline
bool contains_state(const States& s, const State::ptr& p)
{
	foreach( const State::ptr& t, s ) if( p.get() == t.get() ) return true;
	return false;
}
inline
bool contains_state(const States& s, const State *const p)
{
	foreach( const State::ptr& t, s ) if( p == t.get() ) return true;
	return false;
}
inline
bool contains_state(const States& s, const State & p)
{
	foreach( const State::ptr& t, s ) if( &p == t.get() ) return true;
	return false;
}

inline
States& operator+=( States& ss, State::ptr s )
{
	ss.push_back(s);
	return ss;
}
inline
States& operator+=( States& s1, const States& s2 )
{
	s1.insert(s1.end(), s2.begin(), s2.end());
	return s1;
}
inline
States operator+( State::ptr s1, State::ptr s2 )
{
	States s;
	s += s1;
	s += s2;
	return s;
}
inline
States operator+( const States& s1, State::ptr s2 )
{
	States s(s1);
	s += s2;
	return s;
}

class FsmDefinition: public MachineDefinition
{
	friend class FsmInstance;
public:

private:

	States _states;
	State* _start_state;

	States _success_states;
	States _failure_states;

public:

	FsmDefinition(
			const Name& name,
			EventProcessor& processor,
			States states,
			State& s0,
			States succ = States(),
			States fail = States()
	);

	virtual
	~FsmDefinition();

	virtual
	Machine start_instance(const Context& context, Events& private_events)const;

	virtual
	void state( Graph& graph , const Context& context, set<void*>& visited)const;


protected:

	virtual
	Machine process( const Event& event , Machine instance, Events& private_events )const;


	bool is_event_for_interrupt( const Event& event , FsmInstance::FsmMachine& inst )const;
	bool is_event_for_success_stop( const Event& event , FsmInstance::FsmMachine& inst, Events& private_events )const;
	bool is_event_for_failure_stop( const Event& event , FsmInstance::FsmMachine& inst, Events& private_events )const;

	bool is_state_successed( FsmInstance::FsmMachine& inst )const;
	bool is_state_failure( FsmInstance::FsmMachine& inst )const;


};


}
}
}


#endif /* COGNITEAO_MACHINES_SRC_FSM_DEF_H_ */
