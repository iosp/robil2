

#include "TESTS_COMPILATION.h"
#include "test.h"

#if FSM_TEST

#include <cognitao/Fsm.h>
#include <cognitao/Events.h>

using namespace cognitao;
using namespace machine;
using namespace fsm;

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
		cout<<"[e] matched : "<<stranger<<" for "<<original<<endl;
	}

	virtual
	void send(const Event& original){
		cout<<"[e] send    : "<<original<<endl;
		estream.insert(original);
	};

	virtual
	void on_private(const Event& original){
		cout<<"[e] private : "<<original<<endl;
	};
};


int fsm_main(int, char**) {

	Processor p;
	Context c("main");


	FSM_SIMPLE_STATE(C);
	FSM_SIMPLE_STATE(D);
	FSM_SIMPLE_STATE(E);
	FSM_SIMPLE_STATE(F);

	C += Event("?e3") >> D;
	D += Event("?e4") >> C;

	E += Event("?e6") >> F;
	E += Event("?e5") >> E;

	Machine Bottom1 = Machine( new FsmMachine(p, "Bottom1", C, FsmMachine::NO_STATES(), FsmMachine::NO_STATES() , true) );
	Machine Bottom2 = Machine( new FsmMachine(p, "Bottom2", E, States(F) , FsmMachine::NO_STATES(), true) );

	FSM_MACHINE_STATE(A, Bottom1);
	FSM_MACHINE_STATE(B, Bottom2);
	FSM_SIMPLE_STATE(FIN);

	A += Event("?e1") >> B;
	B += Event("?e2") >> A;
	B += Event("machine_report?_stop_bottom2") >> FIN;

	FSM_MACHINE( Top, p, A, FIN , FsmMachine::NO_STATES() );


#	define PRINT_FSM \
	Top->print(cout, c);\
	cout<<"----------------"<<endl;
#	define SEND_EVENT(N, EV) {\
	cout<<"=== "<<N<<" ==="<<endl;\
	Event e = Event(EV, c); p.send(e); \
	while(estream.empty()==false){ estream.print(cout);\
	Machine old = Top;\
	Top = Top->process(estream.remove(), c);\
	/*if(old.get()!=Top.get()) PRINT_FSM*/ } PRINT_FSM; cout<<"----------------"<<endl;}

	SEND_EVENT(1, "machine_control!start_Top")
	SEND_EVENT(2, "!e1")
	SEND_EVENT(3, "!e2")
	SEND_EVENT(4, "!e3")
	SEND_EVENT(5, "!e4")
	SEND_EVENT(6, "!e1")
	SEND_EVENT(7, "!e5")
	SEND_EVENT(8, "!e6")


	return 0;
}

TestRegistrator fsmr( "fsm", fsm_main );


#endif


