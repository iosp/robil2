
#ifndef COGNITEAO_MACHINES_SRC_MACHINEDEF_H_
#define COGNITEAO_MACHINES_SRC_MACHINEDEF_H_

#include "MachineInstance.h"

namespace cognitao{
namespace machine{


class MachineDefinition
{
	friend class MachineInstance;
public:
	typedef boost::shared_ptr<MachineDefinition> ptr;
	typedef std::string Name;



protected:
	Name _name;
	EventProcessor& _processor;
	EventsInterface _interface;

public:

	MachineDefinition( const Name& name, EventProcessor& processor )
		: _name(name)
		, _processor(processor)
	{

	}

	MachineDefinition( const Name& name, EventProcessor& processor, const EventsInterface& interface )
		: _name(name)
		, _processor(processor)
		, _interface(interface)
	{

	}


	MachineDefinition& self(){ return *this; }
	const MachineDefinition& self()const{ return *this; }

	const Name& name()const
	{
		return _name;
	}
	const EventsInterface& interface()const{ return _interface;}
	EventsInterface& interface(){ return _interface;}

	virtual
	~MachineDefinition(){}

	virtual
	Machine start_instance( const Context& context, Events& private_events )const=0;

	virtual
	void state( Graph& graph , const Context& context, set<void*>& visited)const=0;

protected:

	virtual
	Machine process( const Event& event , Machine instance, Events& private_events )const=0;


};

struct MachineDefinitionRef
{
	MachineDefinition::ptr machine;
	MachineDefinitionRef(){}
	MachineDefinitionRef(MachineDefinition::ptr machine):machine(machine){}
	MachineDefinitionRef(MachineDefinition* machine):machine( MachineDefinition::ptr(machine) ){}
};

}
}


#endif /* COGNITEAO_MACHINES_SRC_MACHINEDEF_H_ */
