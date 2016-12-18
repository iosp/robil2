

#include "TESTS_COMPILATION.h"
#include "test.h"

#if FSM_TEST

#include <cognitao/machine/fsm/Fsm.h>
#include <cognitao/machine/Events.h>
#include <cognitao/machine/parallel/Parallel.h>

using namespace cognitao;
using namespace machine;
using namespace fsm;
using namespace parallel;

class EventsStream{
public:
	list<Event> events;
	bool empty()const{ return events.empty();}
	Event remove(){ Event e = events.front(); events.pop_front(); return e; }
	void insert(const Event& e){ events.push_back(e); }
	void print(ostream& out)const{
		out<<"( "; string d="";
		foreachitem(const Event& e, events){
			out<<d<<e; d=", ";
		}
		out<<" )"<<endl;
	}
};
EventsStream estream;

class Processor:public EventProcessor{
public:

	virtual
	void on_match(const Event& original, const Event& stranger){
		cout<<"[e]		matched : "<<stranger<<" for "<<original<<endl;
	}

	virtual
	void send(const Event& original){
		cout<<"[e]		send    : "<<original<<endl;
		estream.insert(original);
	};

	virtual
	void on_private(const Event& original){
		cout<<"[e]		private : "<<original<<endl;
	};
};

Machine send_event( Machine i, const Event& e )
{
	std::cout<<"[i] new event "<<e<<" for "<<i->name()<<std::endl;
	Events es( e );
	while(not es.empty()){
		Event e=es.pop();
		if(not i) return i;
		i = i->process( e, es );
	}
	return i;
}
Machine start_machine( MachineDefinition& def, const Context& context )
{
	Events es;
	Machine i = def.start_instance( context, es );
	while(not es.empty()){
		Event e=es.pop();
		if(not i) return i;
		i = i->process( e, es );
	}
	return i;
}

int fsm_main(int, char**) {

	Processor p;
	Context c("main");

	MachineDefinitionRef mref;
	MachineDefinitionRef m1ref;

	//-----------
	State::ptr s1( new State("s1") );
	State::ptr s2( new State("s2") );
	State::ptr s3( new StateMachine("s3", m1ref) );

	States states;
	states.push_back(s1);
	states.push_back(s2);
	states.push_back(s3);

	//---- ----
	s1->add_condition(Condition::ptr( new Transition( Event("?a"), Events(), *s2 ) ));
	s1->add_condition(Condition::ptr( new Transition( Event("?e"), Events(), *s3 ) ));
	//----
	s2->add_condition(Condition::ptr( new Transition( Event("?d"), Events(), *s1 ) ));
	//----
	s3->add_condition(Condition::ptr( new Transition( Event("?R"), Events(), *s1 ) ));
	//---- ----

	States succ; //succ+=s2;

	mref = MachineDefinitionRef( new FsmDefinition("M", p,    states, *s1, succ, States() ) );

	//-----------
	State::ptr s11( new State("s11") );
	State::ptr s12( new State("s12") );
	State::ptr s13( new StateMachine("s13", mref) );

	States states1;
	states1.push_back(s11);
	states1.push_back(s12);
	states1.push_back(s13);

	s11->add_condition(Condition::ptr( new Transition( Event("?a"), Events(), *s12 ) ));
	s12->add_condition(Condition::ptr( new Transition( Event("?d"), Events(), *s11 ) ));
	s11->add_condition(Condition::ptr( new Transition( Event("?h"), Events(), *s13 ) ));

	m1ref = MachineDefinitionRef( new FsmDefinition("M1", p,    states1, *s11) );

	//-------------
	//mref.machine->interface().interrupt_command += Event("?h");


	Graph g; set<void*> vs;
	mref.machine->state(g, c, vs);

	foreach( const Graph::Nodes::value_type& v, g.nodes )
	{
		cout<<"node : "<<v.second->id()<<endl;
	}

	SimplePrinter printer;
	printer.printers["fsm"] = boost::shared_ptr<SimplePrinterMachine>( new SimplePrinterForFsm() );

	printer.print( cout, g.search_root() );

	Machine i;

	goto STOP_TEST;

	i = start_machine( *mref.machine , c );
	if( not bool( i=send_event(i, Event("!a")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!b")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!c")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!d")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!e")) ) ) goto STOP_TEST;

	if( not bool( i=send_event(i, Event("!a")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!b")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!c")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!d")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!e")) ) ) goto STOP_TEST;

	cout<<"----------"<<endl;
	if( not bool( i=send_event(i, Event("!h")) ) ) goto STOP_TEST;
	if( not bool( i=send_event(i, Event("!a")) ) ) goto STOP_TEST;

	cout<<"----------"<<endl;
	if( not bool( i=send_event(i, Event("!R")) ) ) goto STOP_TEST;
	cout<<"SUCCESS ";

STOP_TEST:
	cout<<"STOP"<<endl;

	return 0;
}

int stop_condition( const parallel::Statistic& stat )
{
	cout<<"STAT: "<<stat.total_count<<", "<<stat.successed_count<<", "<<stat.failures_count<<endl;
	return 1;
}

int parallel_main(int, char**) {

	Processor p;
	Context c("main");

	MachineDefinitionRef _s;
	MachineDefinitionRef _m;
	MachineDefinitionRef _a;
	MachineDefinitionRef _b;
	MachineDefinitionRef _c;

	State::ptr a1( new State("a1") );
	State::ptr b1( new StateMachine("b1", _m) );
	State::ptr a11( new State("a11") );
	State::ptr b11( new State("b11") );
	State::ptr a12( new State("a12") );
	State::ptr a13( new State("a13") );

	a1->add_condition(Condition::ptr( new Transition(Event("?x"), Events(), *b1 ) ) );
	b1->add_condition(Condition::ptr( new Transition(Event("?z"), Events(), *a1 ) ) );

	a11->add_condition(Condition::ptr( new Transition(Event("?x"), Events(), *b11 ) ) );
	b11->add_condition(Condition::ptr( new Transition(Event("?y"), Events(), *a11 ) ) );

	a12->add_condition(Condition::ptr( new Transition(Event("?x"), Events(), *a12 ) ) );

	States s_states;
	s_states += a1+b1;

	States a_states;
	a_states += a11+b11;

	States b_states;
	b_states += a12;

	States c_states;
	c_states += a13;

	MachineList m_machines;
	m_machines.add( _a );
	m_machines.add( _b );
	m_machines.add( _c );
	m_machines.add( _s );

	_s = MachineDefinitionRef( new FsmDefinition("@s", p, s_states, *a1) );
	_a = MachineDefinitionRef( new FsmDefinition("@a", p, a_states, *a11, States()+a11) );
	_b = MachineDefinitionRef( new FsmDefinition("@b", p, b_states, *a12, States()+a12) );
	_c = MachineDefinitionRef( new FsmDefinition("@c", p, c_states, *a13) );
	_m = MachineDefinitionRef( new ParallelDefinition("@m", p, m_machines, stop_condition) );


	Machine i;

	Graph g; set<void*> vs;
	_s.machine->state(g, c, vs);

	foreach( const Graph::Nodes::value_type& v, g.nodes )
	{
		cout<<"node : "<<v.second->id()<<endl;
	}

	SimplePrinter printer;
	printer.printers["fsm"] = boost::shared_ptr<SimplePrinterMachine>( new SimplePrinterForFsm() );
	printer.printers["parallel"] = boost::shared_ptr<SimplePrinterMachine>( new SimplePrinterForParallel() );

	printer.print( cout, g.search_root() );

	goto STOP_TEST;

	i = start_machine( *_s.machine , c );
	cout<<"--- ---1"<<endl;
	if( not bool( i=send_event(i, Event("!x")) ) ) goto STOP_TEST;
	cout<<"---"<<endl;
	if( not bool( i=send_event(i, Event("!y")) ) ) goto STOP_TEST;
	cout<<"--- ---2"<<endl;
	if( not bool( i=send_event(i, Event("!x")) ) ) goto STOP_TEST;
	cout<<"---"<<endl;
	if( not bool( i=send_event(i, Event("!y")) ) ) goto STOP_TEST;
	cout<<"--- ---3"<<endl;
	if( not bool( i=send_event(i, Event("!x")) ) ) goto STOP_TEST;
	cout<<"---"<<endl;
	if( not bool( i=send_event(i, Event("!y")) ) ) goto STOP_TEST;
	cout<<"--- ---4"<<endl;
	if( not bool( i=send_event(i, Event("!z")) ) ) goto STOP_TEST;
	cout<<"--- ---5"<<endl;

	cout<<"SUCCESS ";
STOP_TEST:
	cout<<"STOP"<<endl;

	return 0;
}


int graph_main(int, char**) {

	using namespace graph;

	Graph g;
	Node* root = g.node("root");

	g.node("a");
	g.node("b");
	g.node("c");
	g.node("d");
	g.node("e");

	g.edge( g.node("a"), g.node("b"), "ab", "next" );
	g.edge( g.node("b"), g.node("c"), "bc", "next" );
	g.edge( g.node("b"), g.node("d"), "bd", "alloc" );
	g.edge( g.node("d"), g.node("e"), "de", "next" );
	g.edge( root, g.node("a"), "->a", "alloc");

	XmlPrinter printer;
	printer.print( cout, root )<< endl;
	return 0;
}

TestRegistrator fsmr( "fsm", fsm_main );
TestRegistrator parr( "parallel", parallel_main );

//TestRegistrator fsmr( "graph", graph_main );


#endif


