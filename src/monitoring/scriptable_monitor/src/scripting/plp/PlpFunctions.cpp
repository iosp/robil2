
#include <scriptable_monitor/PlpFunctions.h>
#include <scriptable_monitor/CustomFunctionsSource.h>
#include <scriptable_monitor/PlpModule.h>

#include <map>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

using namespace std;
using namespace boost;
using namespace boost::posix_time;

namespace {
	recursive_mutex global_mtx;
	#define lock_global recursive_mutex::scoped_lock locker(global_mtx);
	map<string,double> global_double;
	map<string,string> global_string;
	map<string,int> global_int;
}

FUNCTION(get_global_var)
{
	lock_global
	string name = input.get<string>(0);
	if(global_double.find(name)==global_double.end()){
		output.set<double>(0);
		return;
	}
	output.set<double>(global_double.at(name));
}

FUNCTION(set_global_var)
{
	lock_global
	string name = input.get<string>(0);
	double value = input.get<double>(1);
	global_double[name]=value;
	output.set<int>(0);
}

FUNCTION(get_global_var_str)
{
	lock_global
	string name = input.get<string>(0);
	if(global_string.find(name)==global_string.end()){
		output.set<string>("");
		return;
	}
	output.set<string>(global_string.at(name));
}

FUNCTION(set_global_var_str){
	lock_global
	string name = input.get<string>(0);
	string value = input.get<string>(1);
	global_string[name]=value;
	output.set<int>(0);
}

FUNCTION(get_global_var_int)
{
	lock_global
	string name = input.get<string>(0);
	if(global_int.find(name)==global_int.end()){
		output.set<int>(0);
		return;
	}
	output.set<int>(global_int.at(name));
}

FUNCTION(set_global_var_int){
	lock_global
	string name = input.get<string>(0);
	int value = input.get<int>(1);
	global_int[name]=value;
	output.set<int>(0);
}


FUNCTION(Now)
{
	output.set<double>(system_time_seconds());
}

FUNCTION(get_module_status)
{
	string name = input.get<string>(0);
	output.set<string>(PlpModule::status(name));
}

FUNCTION(remove_script)
{
	//cout<<"[d] remove_script"<<endl;
	string mdlname = input.get<string>(0);
	string scrname = input.get<string>(1);
	PlpModule::stop_script(mdlname, scrname);
	output.set<int>(0);
}




