
#ifndef COGNITEAO_MACHINES_SRC_ftt_INSTANCE_H_
#define COGNITEAO_MACHINES_SRC_ftt_INSTANCE_H_

#include "../Machine.h"

namespace cognitao{
namespace machine{
namespace ftt{

class FttDefinition;
class Task;
class Condition;

class FttInstance: public MachineInstance
{
	friend class FttDefinition;
	friend class Task;

	Task& _ftt_task;
	const Context _super_context;

public:
	typedef boost::shared_ptr<const FttInstance> FttMachine;
	static FttMachine cast( Machine m ){ return boost::static_pointer_cast<const FttInstance>(m); }

	FttInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context, Task& task );
	FttInstance( const FttInstance& inst, Task& task );

	virtual
	~FttInstance();

	const Context& super_context()const{ return _super_context; }

	virtual
	void state( Graph& graph , const Context& context, set<void*>& visited)const;

	Task& ftt_task(){ return _ftt_task; }

	FttInstance::FttMachine process_task( const Event& event, bool& transition_found )const;

	MachineInstance* clone()const;

	FttInstance::FttMachine  on_task_start()const;
	FttInstance::FttMachine  on_task_abort()const;
	FttInstance::FttMachine  on_task_success()const;
	FttInstance::FttMachine  on_task_fail()const;
	FttInstance::FttMachine  on_task_transition( const Event& event, const Task& task )const;

	FttInstance::FttMachine  on_machine_start()const;
	FttInstance::FttMachine  on_machine_abort()const;
	FttInstance::FttMachine  on_machine_success(Events& private_events)const;
	FttInstance::FttMachine  on_machine_fail(Events& private_events)const;

	virtual
	string name()const;

};
//typedef boost::shared_ptr< const MachineInstance > Machine;

}
}
}


#endif /* COGNITEAO_MACHINES_SRC_FSM_INSTANCE_H_ */
