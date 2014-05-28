/*
 * MEvent.h
 *
 *  Created on: May 19, 2014
 *      Author: dan
 */

#ifndef MEVENT_H_
#define MEVENT_H_

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


class MEvent {
public:
	typedef shared_ptr<MEvent> Ptr;
	class Parameter{public: virtual ~Parameter(){} };
	class Queue{
	protected:
		list<Ptr> events;
		mutex m;
		condition_variable on_new;
		set<Queue*> ch;
		Queue* parent;
	public:
		Queue(Queue& prn):parent(&prn){
			if(parent){
				parent->add_ch(this);
			}
		}
		Queue():parent(0){
			if(parent){
				parent->add_ch(this);
			}
		}
		virtual ~Queue(){
			if(parent){
				mutex::scoped_lock l(m);
				parent->remove_ch(this);
				parent = 0;
			}
			Ptr p = MEvent::TERMINATE();
			add_down(p);
		}
	protected:
		void add_ch(Queue* c){
			mutex::scoped_lock l(m);
			ch.insert(c);
		}
		void remove_ch(Queue* c){
			mutex::scoped_lock l(m);
			ch.erase(c);
		}
	public:
		Ptr get(){
			mutex::scoped_lock l(m);
			if(events.empty()) return Ptr();
			Ptr p = events.front(); events.pop_front();
			return p;
		}
		Ptr wait(){
			mutex::scoped_lock l(m);
			while(not events.empty() and not terminate_function() ){
				on_new.wait(l);
			}
			Ptr p = events.front(); events.pop_front();
			return p;
		}
		void add(Ptr p){
			Queue* _p;
			{
				mutex::scoped_lock l(m);
				_p=parent;
			}
			if(_p){ _p->add(p);}
			else{
				add_down(p);
			}
		}
		void add(MEvent e){
			Ptr p = Ptr(new MEvent(e));
			add(p);
		}
	protected:
		void add_down(Ptr p){
			mutex::scoped_lock l(m);
			events.push_back(p);
			on_new.notify_one();
			BOOST_FOREACH(Queue* i, ch) i->add_down(p);
		}
	};

public:
	MEvent(string name):_name(name),_parameter(0){}
	MEvent(string name, Parameter* param):_name(name),_parameter(param){}
	MEvent(MEvent& e):_name(e._name),_parameter(e._parameter){e._parameter=0;}
	virtual ~MEvent(){
		if(_parameter){ delete _parameter; _parameter=0; }
	}

	static Ptr TERMINATE(){
		static Ptr term = Ptr(new MEvent("____TERMINATOR"));
		return term;
	}

public:
	template <class T>
	T* parameter(){ if(_parameter==0) _parameter = new T(); return static_cast<T>(_parameter); }
	template <class T>
	T* parameter()const{ return static_cast<T>(_parameter); }

	string name()const{ return _name; }

	static Queue& globalQueue(){
		static Queue q;
		return q;
	}

protected:
	string _name;
	Parameter * _parameter;
public:
	static function<bool(void)> terminate_function;

};

#endif /* MEVENT_H_ */
