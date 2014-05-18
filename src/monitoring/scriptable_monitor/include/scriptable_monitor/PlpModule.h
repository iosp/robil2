/*
 * PlpModule.h
 *
 *  Created on: May 18, 2014
 *      Author: dan
 */

#ifndef PLPMODULE_H_
#define PLPMODULE_H_

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

class PlpModule {
public:
	PlpModule();
	virtual ~PlpModule();
	static map<string,PlpModule>& get_all_modules(){
		static map<string,PlpModule> modules;
		return modules;
	}
	static recursive_mutex& mutex(){ static recursive_mutex m; return m; }
	#define PlpModuleLock boost::recursive_mutex::scoped_lock plp_locker(PlpModule::mutex());

	string status()const{ return _status; }
	static string status(string name){
		PlpModuleLock
		string status = "stop";
		if(get_all_modules().count(name))
			status = get_all_modules().at(name).status();
		return status;
	}

	void set_status(string st){ _status = st; }
	static void status(string name, string st){
		PlpModuleLock
		string status = "stop";
		get_all_modules()[name].set_status(st);
	}

private:
	string _status;
};


inline double system_time_seconds(){
	ptime time_t_epoch(boost::gregorian::date(1970,1,1));
	ptime now = microsec_clock::local_time();
	time_duration diff = now - time_t_epoch;
	long x = diff.total_milliseconds();
	return (x*0.001);
}

#endif /* PLPMODULE_H_ */
