

#include "PlpModule.h"
#include <iostream>
#include <sstream>
#include <vector>
//#include <map>
//#include <list>
//#include <set>
//#include <algorithm>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
//#include <boost/shared_ptr.hpp>
//#include <boost/function.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>
//#include <boost/thread/recursive_mutex.hpp>

using namespace std;
using namespace boost;
//using namespace boost::posix_time;

ostream& operator<<(ostream& out, const PlpModule& m){
//	out<<m._plp_text<<endl;
	for(map<string,string>::const_iterator i=m._plp_map.begin();i!=m._plp_map.end();i++){
		out<<i->first<<" = "<<i->second<<endl;
	}
	return out;
}


PlpModule::PlpModule(string filename){
	ifstream file(filename.c_str());
	if(not file){
		throw Exception_FileLoadProblem();
	}
	load_plp(file);
	raise(EVENT_MODULE_START, this);
}
PlpModule::PlpModule(istream& stream){
	load_plp(stream);
	raise(EVENT_MODULE_START, this);
}

PlpModule::~PlpModule(){
	raise(EVENT_MODULE_STOP, this);
}

PlpModule::Goal::Goal(PlpModule* plp):plp(plp){
	for(map<string,string>::const_iterator i=plp->_plp_map.begin();i!=plp->_plp_map.end();i++){
		if(starts_with(i->first, "Goals:")) _goal_map[i->first.substr(i->first.find(":")+1)] = (i->second);
	}
	plp->raise(EVENT_GOAL_ACHIEV_START, plp);
}
PlpModule::Goal::~Goal(){
	plp->raise(EVENT_GOAL_ACHIEV_STOP, plp);
}

void PlpModule::load_plp(istream& stream){
	stringstream text;
	while(stream.eof()==false){
		char c; stream.read(&c,1);
		text<<c;
	}
	_plp_text = text.str();
	parse();
	if(plp_name().empty()){
		throw Exception_PlpHasNoName();
	}
}

string PlpModule::plp_name()const{
	if(_plp_map.find("PLP:Name")==_plp_map.end()) return "";
	return _plp_map.at("PLP:Name");
}
string PlpModule::plp_type()const{
	if(_plp_map.find("PLP:Type")==_plp_map.end()) return "";
	return _plp_map.at("PLP:Type");
}
bool PlpModule::plp_is_repeated()const{
	if(_plp_map.find("Goals:Repeat")==_plp_map.end()) return false;
	return to_upper_copy(_plp_map.at("Goals:Repeat")) == "TRUE";
}

list<PlpModule::EventCallback> PlpModule::events;


struct KeyValue{
	string key;
	string value;
	string section;
	string subsection;
	string full_key()const{
		stringstream s;
		if(section.empty()==false){
			s<<section<<":";
			if(subsection.empty()==false){
				s<<subsection<<":";
			}
		}
		s<<key;
		return s.str();
	}
};
KeyValue parse(string line){
	trim(line);
	vector<string> kv;
	split(kv, line, boost::is_any_of(":"));
	if(kv.size()<2)kv.push_back("");
	if(kv.size()<2)return KeyValue();
	trim(kv[0]); trim(kv[1]);
	KeyValue r={kv[0],kv[1]};
	return r;
}
void parse(KeyValue& kv, string line){
	KeyValue r = parse(line);
	kv.key=r.key;
	kv.value=r.value;
}

void check_if_plp_section(KeyValue& kv){
	if(kv.key=="PLP"){
		kv.section="PLP";kv.subsection="";kv.key="Name";
	}else
	if(kv.key=="Goal repeat"){
		kv.section="Goals";kv.subsection="";kv.key="Repeat";
	}else
	if(kv.key=="Variables"){
		kv.section="Variables";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Parameters"){
		kv.section="Parameters";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Preconditions"){
		kv.section="Preconditions";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Concurrent conditions"){
		kv.section="Concurrent conditions";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Concurrent modules"){
		kv.section="Concurrent modules";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Goals"){
		kv.section="Goals";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Side effects"){
		kv.section="Side effects";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Failure modes"){
		kv.section="Failure modes";kv.subsection="";kv.key="Counter";
	}

	if(kv.section=="Variables" or kv.section=="Parameters" or kv.section=="Concurrent modules" or kv.section=="Failure modes"){
		if(kv.key=="Name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Resources"){
		if(kv.key=="Resource"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Preconditions"){
		if(kv.key=="Precondition name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Concurrent conditions"){
		if(kv.key=="Condition name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Goals"){
		if(kv.key=="Goal name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Side effects"){
		if(kv.key=="Parameter"){
			kv.subsection=kv.value;
		}
	}
}

void PlpModule::parse(){
	vector<string> lines;
	split( lines, _plp_text, boost::is_any_of("\n"));
	KeyValue kv;
	BOOST_FOREACH(string line, lines){
		trim(line);
		::parse(kv,line);
		if(kv.key.empty())continue;
		check_if_plp_section(kv);
		//cout<<line<<" : "<< pref+">"+kv.key << " = "<<kv.value<<endl;
		_plp_map[kv.full_key()]=kv.value;
	}
}


