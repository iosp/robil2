
#ifndef COGNITEAO_MACHINES_SRC_FSM_INSTANCE_H_
#define COGNITEAO_MACHINES_SRC_FSM_INSTANCE_H_

#include "../Machine.h"

namespace cognitao{
namespace machine{
namespace fsm{

class FsmDefinition;
class State;
class Condition;

class FsmInstance: public MachineInstance
{
	friend class FsmDefinition;
	friend class State;

	State& _fsm_state;
	const Context _super_context;

public:
	typedef boost::shared_ptr<const FsmInstance> FsmMachine;
	static FsmMachine cast( Machine m ){ return boost::static_pointer_cast<const FsmInstance>(m); }

	FsmInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context, State& state );
	FsmInstance( const FsmInstance& inst, State& state );

	virtual
	~FsmInstance();

	const Context& super_context()const{ return _super_context; }

	virtual
	void state( Graph& graph , const Context& context, set<void*>& visited)const;

	State& fsm_state(){ return _fsm_state; }

	FsmInstance::FsmMachine process_state( const Event& event, bool& transition_found )const;

	MachineInstance* clone()const;

	FsmInstance::FsmMachine  on_state_start()const;
	FsmInstance::FsmMachine  on_state_abort()const;
	FsmInstance::FsmMachine  on_state_success()const;
	FsmInstance::FsmMachine  on_state_fail()const;
	FsmInstance::FsmMachine  on_state_transition( const Event& event, const State& state )const;

	FsmInstance::FsmMachine  on_machine_start()const;
	FsmInstance::FsmMachine  on_machine_abort()const;
	FsmInstance::FsmMachine  on_machine_success(Events& private_events)const;
	FsmInstance::FsmMachine  on_machine_fail(Events& private_events)const;

	virtual
	string name()const;

};
//typedef boost::shared_ptr< const MachineInstance > Machine;

}
}
}


#endif /* COGNITEAO_MACHINES_SRC_FSM_INSTANCE_H_ */
