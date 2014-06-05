/*
 * Plp.h
 *
 *  Created on: May 29, 2014
 *      Author: dan
 */

#ifndef PlpCpp_H_
#define PlpCpp_H_

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <list>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;

class Plp{
	friend ostream& operator<<(ostream& out, const Plp& m);
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
	typedef boost::function<void(EVENT,const Plp*)> EventCallback;

public:
	class Iteration{
		friend class Plp;
		Iteration(Plp* plp);
		Plp* plp;
		map<string,string> _goal_map;
	public:
		virtual ~Iteration();
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
	Plp(string filename);
	Plp(istream& stream);
	virtual ~Plp();

	void load_plp(istream& stream);
	void parse();
	string plp_name()const;
	string plp_type()const;
	bool plp_is_repeated()const;

	static void subscribe(EventCallback event){
		events.push_back(event);
	}
	static void raise(EVENT type, const Plp* _this){
		BOOST_FOREACH(EventCallback e, events){
			e(type, _this);
		}
	}

	Iteration goal_achievement(){
		if(plp_is_repeated()==false){
			throw Exception_PlpIsNotRepeatable();
		}
		iterations_counter++;
		return Iteration(this);
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

	size_t iterations()const{ return iterations_counter; }

private:
	string _plp_text;
	map<string,string> _plp_map;
	static list<EventCallback> events;
	size_t iterations_counter;
};

ostream& operator<<(ostream& out, const Plp& m);




#endif /* PlpCpp_H_ */
