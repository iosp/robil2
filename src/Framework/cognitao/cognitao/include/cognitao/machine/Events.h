/*
 * Events.h
 *
 *  Created on: Mar 4, 2015
 *      Author: dan
 */

#ifndef COGNITEAO_MACHINES_SRC_EVENTS_H_
#define COGNITEAO_MACHINES_SRC_EVENTS_H_

#include "core.h"
#include "Event.h"
#include "EventProcessor.h"

namespace cognitao{
namespace machine{

class Events: public Printable{
protected:
	typedef list<Event> Collection;
	Collection _events;

	typedef Collection::const_iterator const_iterator;
	typedef Collection::iterator iterator;

public:

	const list<Event>& events()const{ return _events; }
	const_iterator begin()const{ return _events.begin(); }
	const_iterator end()const{ return _events.end(); }
	iterator begin(){ return _events.begin(); }
	iterator end(){ return _events.end(); }
	const Event& front()const{ return _events.front(); }
	const Event& back()const{ return _events.back(); }

	void add(const Event& event){
		_events.push_back(event);
	}

	Events matches(const Event& event, const Context& _context)const;

	Events search(const Event::Direction& dir, const Context& _context)const;

	Events()
	{

	}
	Events(const Event& e)
	{
		add(e);
	}
	Events(const Events& e)
		:_events(e._events)
	{

	}
	Events(const list<Event>& e)
		:_events(e)
	{

	}
	Events(const vector<Event>& e)
		:_events(e.begin(), e.end())
	{

	}
	Events(const set<Event>& e)
		:_events(e.begin(), e.end())
	{

	}

	Events operator+(const Event& e)const{
		Events s(*this);
		s.add(e);
		return s;
	}
	Events operator+(const Events& e)const{
		Events s(*this);
		s+=e;
		return s;
	}

	void operator+=(const Event& e){
		add(e);
	}

	void operator+=(const Events& es){
		_events.insert(_events.end(), es._events.begin(), es._events.end());
	}

	Event pop(){
		Event e = _events.front();
		_events.pop_front();
		return e;
	}
	void print(ostream& out, const Context& context)const;
	void print_multilines(ostream& out, const Context& context)const;

	bool empty()const{ return _events.empty();}
	operator bool()const{ return _events.empty()==false; }
	string str(const Context& c)const{ stringstream s; print(s,c); return s.str(); }

	Events replace_context( const Context& context )const;
};

inline
Events operator,(const Event& e1, const Event& e2){
	return Events(e1)+e2;
}

inline
Events operator,(const Events& e1, const Event& e2){
	return Events(e1)+e2;
}

inline
Events operator+(const Event& ee, const Event& e){ Events r(ee); r.add(e); return r; }

inline
void EventProcessor::send(const Events& original)
{
	BOOST_FOREACH( const Event& e, original.events() ) send(e);
}

inline
void EventProcessor::on_private(const Events& original)
{
	BOOST_FOREACH( const Event& e, original.events() ) on_private(e);
}

}
}


#endif /* MACHINES_SRC_EVENTS_H_ */
