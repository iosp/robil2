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

PlpModule::PlpModule()
{

}

PlpModule::~PlpModule() {

}


void PlpModule::start(){
	for(vector<Info>::iterator i=_scripts.begin();i!=_scripts.end();i++){
		if(i->params["time"]=="on_start"){
			string s = i->source+"\n"+"_tmp=remove_script("+i->params["module"]+","+i->params["name"]+")\n";
			sh->addScript(s);
		}else
		if(i->params["time"]=="on_stop"){

		}else{
			sh->addScript(i->source);
		}
	}
}

void PlpModule::stop(){

}
void PlpModule::resume(){

}
void PlpModule::pause(){

}

void PlpModule::stop_script(string scr){
	sh->deleteScript(scr);
}





