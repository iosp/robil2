
#ifndef COGNITEAO_MACHINES_SRC_PARALLEL_INSTANCE_H_
#define COGNITEAO_MACHINES_SRC_PARALLEL_INSTANCE_H_

#include "../Machine.h"

namespace cognitao{
namespace machine{
namespace parallel{

class ParallelDefinition;

struct Statistic
{
	int total_count;
	int successed_count;
	int failures_count;
	Statistic():total_count(0),successed_count(0),failures_count(0){}
};

class ParallelInstance: public MachineInstance
{
	friend class ParallelDefinition;

	const Context _super_context;
	Statistic _statistic;

public:
	typedef boost::shared_ptr<const ParallelInstance> PrlMachine;
	typedef boost::shared_ptr<ParallelInstance> EditablePrlMachine;
	static PrlMachine cast( Machine m ){ return boost::static_pointer_cast<const ParallelInstance>(m); }

	ParallelInstance( const MachineDefinition& def_machine, EventProcessor& processor, const Context& super_context );
	ParallelInstance( const ParallelInstance& inst );

	virtual
	~ParallelInstance();

	const Context& super_context()const{ return _super_context; }

	virtual
	void state(  Graph& graph , const Context& context, set<void*>& visited)const;

	MachineInstance* clone()const;
	string name()const;

	ParallelInstance::PrlMachine  on_machine_start()const;
	ParallelInstance::PrlMachine  on_machine_abort()const;
	ParallelInstance::PrlMachine  on_machine_success(Events& private_events)const;
	ParallelInstance::PrlMachine  on_machine_fail(Events& private_events)const;

};

}
}
}


#endif /* COGNITEAO_MACHINES_SRC_PARALLEL_INSTANCE_H_ */
