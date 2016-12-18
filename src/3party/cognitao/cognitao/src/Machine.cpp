/*
 * MachineInstance.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */


#include <cognitao/machine/Machine.h>

namespace cognitao {
namespace machine {

size_t MachineInstance::global_count=0;


MachineInstance::MachineInstance(
		const MachineDefinition& def_machine,
		const Context& context,
		EventProcessor& processor,
		const EventsInterface& interface,
		list<Machine> sub_machines
)
	: inst_id(global_count++)
	, _def_machine(def_machine)
	, _processor(processor)
	, _context(context)
	, _machines(sub_machines)
	, _interface(interface)
{
	//std::cout<<"[MACHINE] Created   :"<<inst_id<<":"<<_context.str() <<" : "<<_def_machine.name()<< endl;
}

MachineInstance::~MachineInstance()
{
	//std::cout<<"[MACHINE] Destroyed :"<<inst_id<<":"<<_context.str() << endl;
}

static list<string> tabs;
void print_tabs( ostream& cout )
{
	BOOST_FOREACH( string s, tabs ) cout<<s;
}

MachineInstance::Machine MachineInstance::process_submachines( const Event& in_event, Events& private_events )const
{
	//tabs.push_back(" --");
	Machine res = THIS;
	list<Machine> updated_machines;
	bool has_new = false;
	foreach( Machine m, res->_machines )
	{
		if(not m) continue;

		//print_tabs(cout);
		//cout<<"***"<<endl;

		//print_tabs(cout);
		//cout<<"[MACHINE] MachineInstance::process_submachines >>> ... ... process("<<in_event<<":"<<m->inst_id<<") :"<<inst_id<<endl;

		Machine rm = m->process(in_event, private_events);

		if(rm)
		{
			//print_tabs(cout);
			//cout<<"[MACHINE] MachineInstance::process_submachines >>> ... ... ... result is machine ("<<in_event<<":"<<m->inst_id<<") :"<<inst_id<<endl;

			updated_machines.push_back(rm);
		}
		else
		{
			//print_tabs(cout);
			//cout<<"[MACHINE] MachineInstance::process_submachines >>> ... ... ... result is empty machine ("<<in_event<<":"<<m->inst_id<<") :"<<inst_id<<endl;
		}
		if( rm.get() != m.get() )
		{
			//print_tabs(cout);
			//cout<<"[MACHINE] MachineInstance::process_submachines >>> ... ... ... change detected ("<<in_event<<":"<<m->inst_id<<") :"<<inst_id<<endl;

			has_new = true;
		}
	}
	if( has_new )
	{
		//print_tabs(cout);
		//cout<<"[MACHINE] MachineInstance::process_submachines >>> ... ... clone machine :"<<inst_id<<endl;

		MachineInstance* _t = res->clone();
		Machine _tt( _t );
		res.swap(_tt);
		_t->_machines = updated_machines;
	}

	//tabs.pop_back();
	return res;
}

MachineInstance::Machine MachineInstance::process( const Event& in_event, cognitao::machine::Events& private_events )const
{
	//tabs.push_back(" ---");

	Events events;
	events += in_event;

	//print_tabs(cout);
	//cout<<"[MACHINE] MachineInstance::process >>> begin("<<in_event<<") :"<<inst_id<<endl;
	//if( inst_id == 5 and in_event.str() == "/!e2" )
	//	cout<<"detected1"<<endl;

	MachineInstance::Machine res = process_submachines( in_event, events );

	while( not events.empty() )
	{
		cognitao::machine::Event e = events.pop();

		//print_tabs(cout);
		//cout<<"[MACHINE] MachineInstance::process >>> ... process("<<e<<") :"<<inst_id<<endl;

		res = _def_machine.process (e,res,private_events);
	}

	//print_tabs(cout);
	//cout<<"[MACHINE] MachineInstance::process >>> end :"<<inst_id<<": res="<<(int)(res?res->inst_id:-1)<<endl;
	//if( inst_id == 2 and not res )
	//	cout<<"detected2"<<endl;

	//tabs.pop_back();
	return res;
}

const Context& MachineInstance::context()const
{
	return _context;
}
const MachineDefinition& MachineInstance::definition()const
{
	return _def_machine;
}
EventProcessor& MachineInstance::processor()const
{
	return _processor;
}
EventsInterface& MachineInstance::interface()
{
	return _interface;
}
const EventsInterface& MachineInstance::interface()const
{
	return _interface;
}

const list<MachineInstance::Machine>& MachineInstance::sub_machines()const
{
	return _machines;
}
void MachineInstance::add_machine( Machine m )
{
	assert( m.get() );
	//std::cout<<"[MACHINE] add_machine("/*<<m->name()<<":"*/<<m->inst_id<<")    :"<<inst_id<<endl;//<<":"<<_context.str() <<" : "<<_def_machine.name() << endl;
	_machines.push_back(m);
}
void MachineInstance::remove_machine( Machine m )
{
	assert( m.get() );
	//std::cout<<"[MACHINE] remove_machine("<<m->name()<<":"<<m->inst_id<<") :"<<inst_id<<endl;//<<":"<<_context.str() <<" : "<<_def_machine.name() << endl;
	for( list<Machine>::iterator i=_machines.begin();i!=_machines.end();i++)
	{
		if( m == *i ){ _machines.erase(i); return; }
	}
}
void MachineInstance::clear_machines()
{
	//std::cout<<"[MACHINE] clear_machines                  :"<<inst_id<<endl;//<<":"<<_context.str() <<" : "<<_def_machine.name() << endl;
	_machines.clear();
}

void MachineInstance::interrupt_submachines()const
{
	//std::cout<<"[MACHINE] interrupt_submachines    :"<<inst_id<<endl;//<<":"<<_context.str() <<" : "<<_def_machine.name() << endl;
	Event e( EventsInterface::machine_command_interrupt(false), context() );
	_processor.on_private( e );
	for( list<Machine>::const_iterator i=_machines.begin();i!=_machines.end();i++)
	{
		if(not *i) continue;
		//std::cout<<"[MACHINE] ....interrupt_submachines::process("<<(*i)->inst_id<<")    :"<<inst_id<<endl;//<<":"<<_context.str() <<" : "<<_def_machine.name() << endl;
		Events pr_evs;
		(*i)->process( e , pr_evs );
	}
}


}
}

