/*
 * PlpModule.h
 *
 *  Created on: May 29, 2014
 *      Author: dan
 */

#ifndef PLPMODULE_H_
#define PLPMODULE_H_

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <list>
//#include <set>
//#include <algorithm>

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

class PlpModule{
	friend ostream& operator<<(ostream& out, const PlpModule& m);
public:
	class Exception{
	public:
		virtual ~Exception(){}
	};
	class Exception_FileLoadProblem:public Exception{};
	class Exception_PlpHasNoName:public Exception{};
	class Exception_PlpIsNotRepeatable:public Exception{};

	enum EVENT{
		EVENT_MODULE_START,
		EVENT_MODULE_STOP,
		EVENT_GOAL_ACHIEV_START,
		EVENT_GOAL_ACHIEV_STOP
	};

private:
	typedef boost::function<void(EVENT,const PlpModule*)> EventCallback;

public:
	class Goal{
		friend class PlpModule;
		Goal(PlpModule* plp);
		PlpModule* plp;
		map<string,string> _goal_map;
	public:
		virtual ~Goal();
		string operator[](string key)const{
			if(_goal_map.find(key)==_goal_map.end()) return "";
			return _goal_map.at(key);
		}
		map<string,string> search(string key)const{
			map<string,string> res;
			for(map<string,string>::const_iterator i=_goal_map.begin();i!=_goal_map.end();i++){
				if(starts_with(i->first, key)) res[i->first] = (i->second);
			}
			return res;
		}
	};

public:
	PlpModule(string filename);
	PlpModule(istream& stream);
	virtual ~PlpModule();

	void load_plp(istream& stream);
	void parse();
	string plp_name()const;
	string plp_type()const;
	bool plp_is_repeated()const;

	static void subscribe(EventCallback event){
		events.push_back(event);
	}
	static void raise(EVENT type, const PlpModule* _this){
		BOOST_FOREACH(EventCallback e, events){
			e(type, _this);
		}
	}

	Goal goal_achievement(){
//		if(plp_is_repeated()==false){
//			throw Exception_PlpIsNotRepeatable();
//		}
		return Goal(this);
	}

	string get_script()const{ return _plp_text; }

	vector<string> search(string key)const{
		vector<string> res;
		for(map<string,string>::const_iterator i=_plp_map.begin();i!=_plp_map.end();i++){
			if(starts_with(i->first, key)) res.push_back(i->second);
		}
		return res;
	}
	string operator[](string key)const{
		if(_plp_map.find(key)==_plp_map.end()) return "";
		return _plp_map.at(key);
	}

private:
	string _plp_text;
	map<string,string> _plp_map;
	static list<EventCallback> events;
};

ostream& operator<<(ostream& out, const PlpModule& m);




#endif /* PLPMODULE_H_ */
