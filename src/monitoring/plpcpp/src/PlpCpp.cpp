

#include <plpcpp/PlpCpp.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

#include <plpcpp/RosPack.h>

using namespace std;
using namespace boost;

namespace plp{

ostream& operator<<(ostream& out, const Module& m){
	for(map<string,string>::const_iterator i=m.loader.plp_map().begin();i!=m.loader.plp_map().end();i++){
		out<<i->first<<" = "<<i->second<<endl;
	}
	return out;
}

Module::Module(string filename)
	:iterations_counter(0)
{
	load_plp(filename);
	raise(EVENT_MODULE_START, this);
}
Module::Module(istream& stream)
	:iterations_counter(0)
{
	load_plp(stream);
	raise(EVENT_MODULE_START, this);
}

Module::~Module(){
	raise(EVENT_MODULE_STOP, this);
}

Module::Iteration::Iteration(Module* plp):plp(plp){
	for(map<string,string>::const_iterator i=plp->loader.plp_map().begin();i!=plp->loader.plp_map().end();i++){
		if(starts_with(i->first, "Goals:")) _goal_map[i->first.substr(i->first.find(":")+1)] = (i->second);
	}
	plp->raise(EVENT_GOAL_ACHIEV_START, plp);
}
Module::Iteration::~Iteration(){
	plp->raise(EVENT_GOAL_ACHIEV_STOP, plp);
}

void Module::load_plp(istream& stream){
	loader.load_plp(stream);
}
void Module::load_plp(string filename){
	loader.load_plp(filename);
}

string Module::plp_name()const{
	return loader.plp_name();
}
string Module::plp_type()const{
	return loader.plp_type();
}
bool Module::plp_is_repeated()const{
	return loader.plp_is_repeated();
}

list<Module::EventCallback> Module::events;



}


