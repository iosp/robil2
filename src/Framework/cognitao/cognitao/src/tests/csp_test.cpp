
#include "TESTS_COMPILATION.h"
#include "test.h"

#if CSP_TEST == 1
//
//#include <cognitao/Csp.h>
//
//using namespace cognitao;
//using namespace machine;
//using namespace csp;
//
//class EventsStream{
//public:
//	list<Event> events;
//	bool empty()const{ return events.empty();}
//	Event remove(){ Event e = events.front(); events.pop_front(); return e; }
//	void insert(const Event& e){ events.push_back(e); }
//	void print(ostream& out)const{
//		out<<"( "; string d="";
//		foreachitem(const Event& e, events){
//			out<<d<<e; d=", ";
//		}
//		out<<" )"<<endl;
//	}
//};
//EventsStream estream;
//
//class Processor:public EventProcessor{
//public:
//
//	virtual
//	void on_match(const Event& original, const Event& stranger){
//		cout<<"[e] matched : "<<stranger<<" for "<<original<<endl;
//	}
//
//	virtual
//	void send(const Event& original){
//		Event e(original);
//		e.change_properties(Event::OUTCOME, e.scope());
//		cout<<"[e] send    : "<<e<<endl;
//		estream.insert(e);
//	};
//
//	virtual
//	void on_private(const Event& original){
//		cout<<"[e] private : "<<original<<endl;
//	};
//
//
//	void on_call(const Event& e){
////		EventProcessor::on_call(e, ctx);
////		Event ee(e, ctx);
////
////		size_t i = ee.text().find('?');
////		if(i == string::npos or i == ee.text().size()-1) return;
////
////		string id = ee.text().substr(i+1);
////		string RESULT = "B";
////		results.push_back(Event(id+"."+RESULT));
//	}
//};



int scp_main(int, char**) {
//
//	Processor p;
//	Context c("main");
//
//	CSP_PROCESS(P);
//	CSP_PROCESS(P1);
//
//
//	P = Event("!e1") >> Event("?e2") >> Event("!e3") >> STOP;
//	P1 = Event("?e4") >> STOP;
//
//	CSP_PROCESS(RES) = (P*P1);
//	RES.set_type(SequenceProcess::ST_AND);
//
//	Machine process = Machine( new CspMachine( p, "test", RES ) );
//
//
//#	define PRINT_CSP \
//		process->print(cout, c);\
//		cout<<"\n----------------"<<endl;
//#	define SEND_EVENT(N, EV) {\
//		cout<<"=== "<<N<<" ==="<<endl;\
//		Event e = Event(EV, c); p.send(e); \
//		while(estream.empty()==false){ estream.print(cout);\
//		Machine old = process;\
//		process = process->process(estream.remove(), c);\
//		/*if(old.get()!=bt.get()){ PRINT_BT; cout<<"----"<<endl; }*/ } PRINT_CSP; cout<<"----------------"<<endl;}
//
//	PRINT_CSP
//	SEND_EVENT(1, "machine_control!start_test");
//	SEND_EVENT(1, "e2");

	return 0;
}

TestRegistrator csptr( "csp", scp_main );

#endif


