
#ifndef COGNITEAO_MACHINES_SRC_MACHINE_INSTANCE_H_
#define COGNITEAO_MACHINES_SRC_MACHINE_INSTANCE_H_

#include "Events.h"
#include "MachineEventsInterface.h"

namespace cognitao{
namespace machine{

using namespace graph;

class MachineDefinition;

class MachineInstance: CONST_SHARED(MachineInstance)
{
public:
	friend class MachineDefinition;
	typedef boost::shared_ptr< const MachineInstance > Machine;

	static size_t global_count;
	size_t inst_id;

protected:

	const MachineDefinition& _def_machine;
	EventProcessor& _processor;
	Context _context;
	list<Machine> _machines;
	EventsInterface _interface;

	MachineInstance(
			const MachineDefinition& def_machine,
			const Context& context,
			EventProcessor& processor,
			const EventsInterface& interface,
			list<Machine> sub_machines = list<Machine>()
	);

public:


	virtual
	~MachineInstance();

	const Context& context()const;
	const MachineDefinition& definition()const;
	EventProcessor& processor()const;
	const list<Machine>& sub_machines()const;

	EventsInterface& interface();
	const EventsInterface& interface()const;

	void add_machine( Machine m );
	void remove_machine( Machine m );
	void clear_machines();

	Machine process( const Event& event , Events& private_events ) const;
	Machine process_submachines( const Event& event , Events& private_events ) const;

	virtual
	void state(  Graph& graph , const Context& context, set<void*>& visited)const=0;

	virtual
	MachineInstance* clone()const=0;

	void interrupt_submachines()const;

	virtual
	string name()const=0;

};
typedef boost::shared_ptr< const MachineInstance > Machine;

}
}


#endif /* COGNITEAO_MACHINES_SRC_MACHINE_INSTANCE_H_ */
