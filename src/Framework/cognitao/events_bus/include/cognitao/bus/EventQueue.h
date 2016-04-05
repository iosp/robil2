/*
 * EventQueue.h
 *
 *  Created on: Nov 5, 2014
 *      Author: dan
 */

#ifndef EVENTQUEUE_H_
#define EVENTQUEUE_H_

#include "Event.h"

namespace cognitao{
namespace bus{

class EventRiser{
public:
	typedef boost::shared_ptr<EventRiser> Ptr;

	virtual
	~EventRiser(){}

	virtual
	void rise(const Event& event)=0;

	EventRiser& operator<<(const Event& e)
	{
		rise(e);
		return *this;
	}
};
class EventQueue:public EventRiser{
public:
	typedef boost::shared_ptr<EventQueue> Ptr;

	mutable mutex m;
	mutable condition_variable new_event;
	size_t events_max_size;
	bool closed;
	list<Event> events;

	static const Event& idleEvent(){
		static const Event ie = Event::IDLE;
		return ie;
	}

	EventQueue* parent;
	typedef set<EventQueue*> Subscribers;
	Subscribers subscribers;
	class Deligate{
	public:
		virtual ~Deligate(){}
		virtual void on_event(const Event& event)=0;
	};
	typedef set<Deligate*> Deligates;
	Deligates deligates;

	thread* idle_timer;

	EventQueue(EventQueue* parent=0,size_t events_max_size=1000);
	EventQueue(EventQueue& parent_ref,size_t events_max_size=1000);
	virtual ~EventQueue();


	void _push_subscribers(const Event& e);
	void _close_subscribers();
	void add(EventQueue* q);
	void remove(EventQueue* q);

	void add(Deligate* deligate);
	void remove(Deligate* deligate);

	void add(Deligate& deligate);
	void remove(Deligate& deligate);

	void on_event(const Event& event);

	void push(const Event& e);
	bool _pop(Event& e);
	bool pop(Event& e);
	void clear();
	bool wait_and_pop(Event& e);
	bool wait_and_pop_timed(Event& e, const time_duration& duration, bool& is_timeout);
	bool wait_for(Event& e);
	bool wait_for_timed(Event& e, const time_duration& duration, bool& is_timeout);
	bool wait_for(set<Event>& e, Event& g);
	bool wait_for_timed(set<Event>& e, Event& g, const time_duration& duration, bool& is_timeout);
	void close_without_remove();
	void close();

	virtual
	void rise(const Event& e);

	void thread_idle_timer(posix_time::time_duration time);
	void run_idle_timer(posix_time::time_duration time);
	void stop_idle_timer();

	bool empty()const;
	size_t size()const;
	bool is_closed()const;
	bool eof()const;

	void set_queue_size( size_t sz);
	size_t current_queue_size()const;

	EventQueue& operator>>(Event& e);

	friend ostream& operator<<(ostream& out, const EventQueue& e);
};

}
}

ostream& operator<<(ostream& out, const cognitao::bus::EventQueue& e);

template<class T>
boost::shared_ptr<T>& operator<<(boost::shared_ptr<T>& out, const cognitao::bus::Event& e){
	(*out) << e; return out;
}

template<class T>
boost::shared_ptr<T>& operator>>(boost::shared_ptr<T>& out, cognitao::bus::Event& e){
	(*out) >> e; return out;
}

namespace cognitao{
namespace bus{

class EventFilter{
public:
    EventQueue& events;
    vector<Event> templates;
    
    EventFilter(EventQueue& events):events(events){}
    
    void add(const Event& e){ templates.push_back(e); }
    void add(const std::string& e){ templates.push_back(Event(e)); }
    
    bool wait_and_pop(Event& e){
        while(true){
            bool res = events.wait_and_pop(e);
            if(not res) return res;
            for(size_t i=0;i<templates.size();i++){
                if( e == templates[i] ) return true;
            }
        }
    }
    bool wait_and_pop_timed(Event& e, const time_duration& duration, bool& is_timeout){
        while(true){
            bool res = events.wait_and_pop_timed(e, duration, is_timeout);
            if(not res) return res;
            for(size_t i=0;i<templates.size();i++){
                if( e == templates[i] ) return true;
            }
        }
    }
};    

}
}

#endif /* EVENTQUEUE_H_ */
