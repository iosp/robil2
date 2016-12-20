
#include "TESTS_COMPILATION.h"
#include "test.h"

#if BT_TEST == 1

#include <cognitao/Bt.h>

using namespace cognitao;
using namespace machine;
using namespace bt;

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
		Event e(original);
		e.change_properties(Event::OUTCOME, e.scope());
		cout<<"[e] send    : "<<e<<endl;
		estream.insert(e);
	};

	virtual
	void on_private(const Event& original){
		cout<<"[e] private : "<<original<<endl;
	};
};

#include <boost/date_time.hpp>
int bt_main(int a, char** aa) {

	Processor p;
	Context c("main");

	BT_TASK( t1, Event("?a1"), Event("?b1"),   Event("!c1"), Event("!d1") );
	BT_TASK( t2, Event("?a2"), Event("?b2"),   Event("!c2"), Event("!d2") );
	BT_TASK( t3, Event("?a3"), Event("?b3"),   Event("!c3"), Event("!d3") );
	BT_TASK( t4, Event("?a4"), Event("?b4"),   Event("!c4"), Event("!d4") );

	BT_BEHAVIOR( mission ) = bt::REPEAT_UNTIL(false, 2, bt::REPEAT_UNTIL(true, 2, bt::NOT(t1)) & t1);// & ( t3 | t2 | t2 | t1 ) & t4;

	BT_MACHINE( low_bt, p, mission );

	BT_MACHINE_TASK( top_mission, low_bt );

	BT_MACHINE( bt, p, top_mission );


#	define PRINT_BT \
		bt->print(cout, c);\
		cout<<"----------------"<<endl;
#	define SEND_EVENT(N, EV) {\
		cout<<"=== "<<N<<" ==="<<endl;\
		Event e = Event(EV, c); p.send(e); \
		while(estream.empty()==false){ estream.print(cout);\
		Machine old = bt;\
		bt = bt->process(estream.remove(), c);\
		/*if(old.get()!=bt.get()){ PRINT_BT; cout<<"----"<<endl; }*/ } PRINT_BT; cout<<"----------------"<<endl;}


	PRINT_BT

	SEND_EVENT(1, "machine_control!start_bt")
	SEND_EVENT(2, "a1")
	SEND_EVENT(3, "a1")
	SEND_EVENT(4, "a1")
	SEND_EVENT(5, "a1")
	SEND_EVENT(6, "a1")
	SEND_EVENT(7, "a1")
	SEND_EVENT(8, "a1")
	SEND_EVENT(9, "a1")


	return 0;
}


TestRegistrator bttr( "bt", bt_main );


#endif

