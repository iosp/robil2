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

namespace plp{

class Module{
	friend ostream& operator<<(ostream& out, const Module& m);
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
	typedef boost::function<void(EVENT,const Module*)> EventCallback;

public:
	class Iteration{
		friend class Module;
		Iteration(Module* plp);
		Module* plp;
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
	Module(string filename);
	Module(istream& stream);
	virtual ~Module();

	void load_plp(istream& stream);
	void parse();
	string plp_name()const;
	string plp_type()const;
	bool plp_is_repeated()const;

	static void subscribe(EventCallback event){
		events.push_back(event);
	}
	static void raise(EVENT type, const Module* _this){
		BOOST_FOREACH(EventCallback e, events){
			e(type, _this);
		}
	}

	Iteration goal_achievement(){
		if(plp_is_repeated()==false){
			throw Exception_PlpIsNotRepeatable();
		}
		iterations_counter++; //cout<<"[i] "<<plp_name()<<" : iteration #"<<iterations_counter<<endl;
		return Iteration(this);
	}

	string get_script()const{ stringstream s; s<<(is_need_header()?_header:"") << "\n" <<_plp_text; return s.str();}
	bool is_need_header()const;

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
	size_t iterations_counter;
	string _header;
	static list<EventCallback> events;

};


ostream& operator<<(ostream& out, const Module& m);

}


#endif /* PlpCpp_H_ */
