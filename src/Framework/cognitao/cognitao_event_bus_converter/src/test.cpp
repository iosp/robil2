/*
 * test.cpp
 *
 *  Created on: Oct 8, 2015
 *      Author: dan
 */

#include <cognitao/events_adapter/FsmEventsAdapter.h>
#include <iostream>



using namespace std;
using namespace cognitao;

std::string str( const machine::Event::Scope& s )
{
	if( s == machine::Event::LOCAL  ) return "LOCAL";
	if( s == machine::Event::GLOBAL ) return "GLOBAL";
	if( s == machine::Event::SUPER  ) return "SUPER";
	if( s == machine::Event::CHILD  ) return "CHILD";
	if( s == machine::Event::PARENT ) return "PARENT";
	return "[x]";
}

std::string str( const bus::Event::Scope& s )
{
	if( s == bus::Event::SCOPE_GLOBAL) return "GLOBAL";
	if( s == bus::Event::SCOPE_UP    ) return "UP";
	if( s == bus::Event::SCOPE_DOWN  ) return "DOWN";
	if( s == bus::Event::SCOPE_CHILD ) return "CHILD";
	if( s == bus::Event::SCOPE_PARENT) return "PARENT";
	return "[x]";
}

std::string str_params( const machine::Event& event )
{
	std::stringstream s;
	//for( size_t i=0;i<event.parameters().size();i++)
	{
		s<<event.parameters();
	}
	return s.str();
}
std::string str_params( const bus::Event& event )
{
	std::stringstream s;
	//for( size_t i=0;i<event.parameters().size();i++)
	{
		s<<event.parameters();
	}
	return s.str();
}

void print( ostream& cout, const cognitao::machine::Event& mevent, const std::string& title )
{
	if(title!=""){
		cout<<"[i] -------------------------"<<endl;
		cout<<"[i] "<<title<<endl;
	}
	cout<<"[i] -------------------"<<endl;
	cout<<"[i] machine event 	is "<<mevent<<endl;
	cout<<"[i] ... name 	 	is "<<mevent.name()<<endl;
	cout<<"[i] ... channel		is "<<mevent.channel()<<endl;
	cout<<"[i] ... scope 	 	is "<<str(mevent.scope())<<endl;
	cout<<"[i] ... context		is "<<mevent.context().str()<<endl;
	cout<<"[i] ... parameters 	is {"<<str_params(mevent)<<"}"<<endl;
}

void print( ostream& cout, const cognitao::bus::Event& bevent, const std::string& title )
{
	if(title!=""){
		cout<<"[i] -------------------------"<<endl;
		cout<<"[i] "<<title<<endl;
	}
	cout<<"[i] -------------------"<<endl;
	cout<<"[i] bus event 		is "<<bevent<<endl;
	cout<<"[i] ... name 		is "<<bevent.name()<<endl;
	cout<<"[i] ... channel		is "<<bevent.channel()<<endl;
	cout<<"[i] ... scope 		is "<<str(bevent.scope())<<endl;
	cout<<"[i] ... source 		is "<<bevent.source()<<endl;
	cout<<"[i] ... context		is "<<bevent.context().full_name()<<endl;
	cout<<"[i] ... parameters 	is {"<<str_params(bevent)<<"}"<<endl;
}

void print( ostream& cout, const std::vector<cognitao::machine::Event>& mevent, const std::string& title )
{
	if(title!=""){
		cout<<"[i] -------------------------"<<endl;
		cout<<"[i] "<<title<<endl;
	}
	for(size_t i=0;i<mevent.size();i++)
	{
		print(cout, mevent[i], "");
	}
}
void print( ostream& cout, const std::vector<cognitao::bus::Event>& bevent, const std::string& title )
{
	if(title!=""){
		cout<<"[i] -------------------------"<<endl;
		cout<<"[i] "<<title<<endl;
	}
	for(size_t i=0;i<bevent.size();i++)
	{
		print(cout, bevent[i], "");
	}

}



struct Block{
	std::string title;
	Block( string t ): title(t){ std::cout<<"START "<<title<<endl;}
	~Block(){ std::cout<<"END "<<title<<endl; }
};


int main( int a, char** aa )
{Block b_program("program");

	{Block b_test("test");

		std::vector<machine::Event> mresult;
		std::vector<bus::Event> bresult;

		{Block b_define("define");

			machine::Event mevent("state_report?exit_r",machine::Context("/a/b/c"));
			print( cout , mevent , "machine event definition" );

			bus::Event bevent("/a/b/c/states./name/f/k(a,b)");
			print( cout , bevent , "bus event definition" );

			{Block b_copy("copy");

				cognitao::events_adapter::FsmEventsAdapter adapter;
				cognitao::events_adapter::EventsAdapter general_adapter;

				{Block b_copy_machine("copy_machine");
					vector<machine::Event> res_m_events;

					general_adapter.on_bus_event( bevent , res_m_events );
					mresult.insert(mresult.end(), res_m_events.begin(), res_m_events.end());

					res_m_events.clear();

					adapter.on_bus_event( bevent , res_m_events );
					mresult.insert(mresult.end(), res_m_events.begin(), res_m_events.end());
				}

				{Block b_copy_bus("copy_bus");
					vector<bus::Event> res_b_events;

					general_adapter.on_machine_event( mevent , res_b_events );
					bresult.insert(bresult.end(), res_b_events.begin(), res_b_events.end());

					res_b_events.clear();

					adapter.on_machine_event( mevent , res_b_events );
					bresult.insert(bresult.end(), res_b_events.begin(), res_b_events.end());
				}
			}

		}


		print( cout , mresult , "machine event result" );
		print( cout , bresult , "bus event result" );


	}
	return 0;
}
