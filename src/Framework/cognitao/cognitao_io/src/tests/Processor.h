/*
 * Processor.h
 *
 *  Created on: Dec 2, 2015
 *      Author: dan
 */

#ifndef SRC_TESTS_PROCESSOR_H_
#define SRC_TESTS_PROCESSOR_H_


#include <iostream>
#include <sstream>
#include <fstream>
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>


using namespace std;

namespace test{
	using namespace cognitao::machine;

	class EventsStream{
	public:
		list<Event> events;
		bool empty()const{ return events.empty();}
		Event remove(){ Event e = events.front(); events.pop_front(); return e; }
		void insert(const Event& e){ events.push_back(e); }
		void print(ostream& out)const{
			out<<"( "; string d="";
			BOOST_FOREACH(const Event& e, events){
				out<<d<<e; d=", ";
			}
			out<<" )"<<endl;
		}
	};

	class Processor: public cognitao::machine::EventProcessor
	{
	public:
		EventsStream& estream;

		Processor( EventsStream& es )
		: estream(es)
		{

		}

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
}




#endif /* SRC_TESTS_PROCESSOR_H_ */
