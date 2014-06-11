/*
 * PlpModule.cpp
 *
 *  Created on: May 18, 2014
 *      Author: dan
 */

#include <scriptable_monitor/PlpModule.h>
#include <scriptable_monitor/monitoring_events/MEvent.h>
#include <scriptable_monitor/ScriptHost.h>

#include <map>
#include <list>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;
using namespace boost::posix_time;



ScriptHost* PlpModule::sh = 0;
map<string, PlpModule::Timer> PlpModule::_timers;

PlpModule::PlpModule()
:_status("created"), _internal_status("created"), _repeated_time(-1)
{

}

PlpModule::~PlpModule() {

}


void PlpModule::start(){
	if(_status == "run") return;

	for(vector<Info>::iterator i=_scripts.begin();i!=_scripts.end();i++){
		if(i->params["time"]=="on_start"){
			string source = i->source;
			string mname = i->params["module"];
			string sname = i->params["name"];
			string s = i->source;
			//s = s + "\n" +"print 'one time script ', '"+sname+"'";
			s = boost::str(boost::format("%s\nremove_script('%s','%s')\n") %s %mname %sname );
			sh->addScript(s);
		}else
		if(i->params["time"]=="on_stop"){

		}else{
			sh->addScript(i->source);//+"\nprint 'check "+i->params["name"]+"'\n");
		}
	}
	if(_repeated_time>0){
		//cout<<"_repeated_time = "<<_repeated_time<<endl;
		start_timer(_module_name+"_"+"repeat", _repeated_time, "report error module "+_module_name+" is timed out.");
	}
	_status = "run";
	_internal_status = "run";
}

void PlpModule::stop(){
	if(_status == "stop") return;
	if(_internal_status == "standby"){
		stop_timer_by_prefix(_module_name);
		_status = "stop";
		_internal_status = "stop";
		return;
	}

	for(vector<Info>::iterator i=_scripts.begin();i!=_scripts.end();i++){
		if(i->params["time"]=="on_start"){
		}else
		if(i->params["time"]=="on_stop"){
			string source = i->source;
			string mname = i->params["module"];
			string sname = i->params["name"];
			string s = i->source;
			//s = s + "\n" +"print 'one time script ', '"+sname+"'";
			s = boost::str(boost::format("%s\nremove_script('%s','%s')\n") %s %mname %sname );
			sh->addScript(s);
		}else{
			stop_script(i->params["name"]);
		}
	}
	stop_timer_by_prefix(_module_name);
	_status = "stop";
	_internal_status = "stop";
}
void PlpModule::resume(){
	if(_status == "stop") return;
	if(_internal_status == "run") return;

	for(vector<Info>::iterator i=_scripts.begin();i!=_scripts.end();i++){
		if(i->params["time"]=="on_start"){
			string source = i->source;
			string mname = i->params["module"];
			string sname = i->params["name"];
			string s = i->source;
			//s = s + "\n" +"print 'one time script ', '"+sname+"'";
			s = boost::str(boost::format("%s\nremove_script('%s','%s')\n") %s %mname %sname );
			sh->addScript(s);
		}else
		if(i->params["time"]=="on_stop"){

		}else{
			sh->addScript(i->source);
		}
	}
	if(_repeated_time>0){
		restart_timer_by_prefix(_module_name+"_"+"repeat");
	}
	_internal_status = "run";
}
void PlpModule::pause(){
	if(_status == "stop") return;
	if(_internal_status == "standby") return;

	for(vector<Info>::iterator i=_scripts.begin();i!=_scripts.end();i++){
		if(i->params["time"]=="on_start"){
		}else
		if(i->params["time"]=="on_stop"){
			string source = i->source;
			string mname = i->params["module"];
			string sname = i->params["name"];
			string s = i->source;
			//s = s + "\n" +"print 'one time script ', '"+sname+"'";
			s = boost::str(boost::format("%s\nremove_script('%s','%s')\n") %s %mname %sname );
			sh->addScript(s);
		}else{
			stop_script(i->params["name"]);
		}
	}

	_internal_status = "standby";
}

void PlpModule::stop_script(string scr){
	sh->deleteScript(scr);
}





