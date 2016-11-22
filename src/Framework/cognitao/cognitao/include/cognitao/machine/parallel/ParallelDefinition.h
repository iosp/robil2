
#ifndef COGNITEAO_MACHINES_SRC_PARALLEL_DEF_H_
#define COGNITEAO_MACHINES_SRC_PARALLEL_DEF_H_

#include "ParallelInstance.h"

namespace cognitao{
namespace machine{
namespace parallel{


class MachineRef{
	MachineDefinition* _def;
	MachineDefinitionRef* _def_ref;

public:
	MachineRef( MachineDefinition& def )
		:_def(&def), _def_ref(0)
	{

	}
	MachineRef( MachineDefinitionRef& def )
		:_def(0), _def_ref(&def)
	{

	}

	MachineDefinition& def_machine()const;
};

class MachineList{
public:
	std::list<MachineRef> machines;
	void add( MachineDefinition& def )
	{
		machines.push_back(MachineRef(def));
	}
	void add( MachineDefinitionRef& def )
	{
		machines.push_back(MachineRef(def));
	}
};


class StopCondition
{
public:
	virtual
	~StopCondition(){}
	virtual
	int test( const Statistic& statistic )const=0;
};

template<class T>
class StopConditionFunction: public StopCondition
{
public:
	T ptr;
	StopConditionFunction( T ptr ):ptr(ptr){}
	int test( const Statistic& statistic )const{ return ptr( statistic ); }
};

namespace stop_conditions{
	static
	int while_all_are_running( const Statistic& stat )
	{
		int s = stat.successed_count, f = stat.failures_count, t = stat.total_count;
		if( t==0 or s+f>0 ) return s>=f ? 1 : -1;
		return 0;
	}
	static
	int while_until_all_successed( const Statistic& stat )
	{
		int s = stat.successed_count, f = stat.failures_count, t = stat.total_count;
		if( t == 0 or t-(s+f) <= 0 or f>0 ) return f>0 ? -1 : 1;
		return 0;
	}

}

class ParallelDefinition: public MachineDefinition
{
	friend class ParallelInstance;

public:

	enum STOP_CONDITION
	{
		WHILE_ALL_RUN,
		WHILE_ALL_RUN_OR_SUCCESS,
		WHILE_ALL_RUN_OR
	};

private:

	MachineList _def_machines;
	boost::shared_ptr<StopCondition> _stop_condition;

public:

	ParallelDefinition(
			const Name& name,
			EventProcessor& processor,
			const MachineList& sub_machines
	);

	template<class STOP_CONDITION>
	ParallelDefinition(
			const Name& name,
			EventProcessor& processor,
			const MachineList& sub_machines,
			STOP_CONDITION stop_cond
	)
		: MachineDefinition( name, processor)
		, _def_machines(sub_machines)
		,_stop_condition( new StopConditionFunction<STOP_CONDITION>(stop_cond) )
	{

	}

	virtual
	~ParallelDefinition();

	virtual
	Machine start_instance(const Context& context, Events& private_events)const;

	virtual
	void state(  Graph& graph , const Context& context, set<void*>& visited)const;


protected:

	virtual
	Machine process( const Event& event , Machine instance, Events& private_events )const;


	bool is_event_for_interrupt( const Event& event , ParallelInstance::PrlMachine& inst )const;
	bool is_event_for_success_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const;
	bool is_event_for_failure_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const;
	bool on_child_stop( const Event& event , ParallelInstance::PrlMachine& inst, Events& private_events )const;



};


}
}
}


#endif /* COGNITEAO_MACHINES_SRC_FSM_DEF_H_ */
