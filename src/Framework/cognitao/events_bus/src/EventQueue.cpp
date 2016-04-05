/*
 * EventQueue.cpp
 *
 *  Created on: Nov 5, 2014
 *      Author: dan
 */

#include <cognitao/bus/events_bus.h>

namespace cognitao{
namespace bus{

EventQueue::EventQueue(EventQueue* parent,size_t events_max_size):
	events_max_size(events_max_size),closed(false),parent(parent),idle_timer(0)
{
	if(parent) parent->add(this);
}
EventQueue::EventQueue(EventQueue& parent_ref,size_t events_max_size):
	events_max_size(events_max_size),closed(false),parent(&parent_ref),idle_timer(0)
{
	parent->add(this);
}
EventQueue::~EventQueue(){
	stop_idle_timer();
	close();
}


void EventQueue::_push_subscribers(const Event& e){
	for(Subscribers::iterator i=subscribers.begin();i!=subscribers.end();i++){
		(*i)->push(e);
	}
}
void EventQueue::_close_subscribers(){
	for(Subscribers::iterator i=subscribers.begin();i!=subscribers.end();i++){
		(*i)->close_without_remove();
	}
	subscribers.clear();
}
void EventQueue::add(EventQueue* q){
	if(not q) return;
	mutex::scoped_lock l(m);
	subscribers.insert(q);
}
void EventQueue::remove(EventQueue* q){
	mutex::scoped_lock l(m);
	subscribers.erase(q);
}

void EventQueue::add(Deligate* deligate){
	if(not deligate) return;
	mutex::scoped_lock l(m);
	deligates.insert(deligate);
}
void EventQueue::remove(Deligate* deligate){
	mutex::scoped_lock l(m);
	deligates.erase(deligate);
}

void EventQueue::add(Deligate& deligate){
	add(&deligate);
}
void EventQueue::remove(Deligate& deligate){
	remove(&deligate);
}

void EventQueue::on_event(const Event& event){
	for(Deligates::iterator i=deligates.begin();i!=deligates.end();i++){
		(*i)->on_event(event);
	}
}

void EventQueue::set_queue_size( size_t sz)
{
	mutex::scoped_lock l(m);
	events_max_size = sz;
	while(events_max_size > events.size()){
		events.pop_front();
	}
}
size_t EventQueue::current_queue_size()const
{
	mutex::scoped_lock l(m);
	return events_max_size;
}

void EventQueue::push(const Event& e){
	mutex::scoped_lock l(m);
	if(closed) return;
	if(e.is_template()) return;
	if(events_max_size>0){
		if(events.size()>=events_max_size)
			events.pop_front();
		if(e == idleEvent()){
			if(events.empty() or events.back()!=idleEvent())
				events.push_back(e);
		}else{
			events.push_back(e);
		}
	}
	on_event(e);
	_push_subscribers(e);
	new_event.notify_one();
}
bool EventQueue::_pop(Event& e){
	if(events.empty()) return false;
	e = events.front();
	events.pop_front();
	return true;
}
bool EventQueue::pop(Event& e){
	mutex::scoped_lock l(m);
	if(closed) return false;
	return _pop(e);
}
void EventQueue::clear(){
	mutex::scoped_lock l(m);
	if(closed) return;
	events.clear();
}
bool EventQueue::wait_and_pop(Event& e){
	mutex::scoped_lock l(m);
	if(closed) return false;
	while(events.empty()){
		new_event.wait(l);
		if(closed) return false;
	}
	return _pop(e);
}
bool EventQueue::wait_and_pop_timed(Event& e, const time_duration& duration, bool& is_timeout){
	mutex::scoped_lock l(m);
	if(closed) return false;
	while(events.empty()){
		bool is_not_timeout = new_event.timed_wait(l, duration);
		is_timeout = not is_not_timeout;
		if(closed) return false;
		if(is_timeout) return false;
	}
	return _pop(e);
}
bool EventQueue::wait_for_timed(Event& e, const time_duration& duration, bool& is_timeout){
	Event new_e;
	while(true)
	{
		mutex::scoped_lock l(m);
		if(closed) return false;
		while(events.empty()){
			bool is_not_timeout = new_event.timed_wait(l, duration);
			is_timeout = not is_not_timeout;
			if(closed) return false;
			if(is_timeout) return false;
		}
		bool r = _pop(new_e);
		if( not r ) return false;
		if( e.match(new_e) ){ e = new_e; return true; }
	}
}
bool EventQueue::wait_for(Event& e){
	Event new_e;
	while(true)
	{
		mutex::scoped_lock l(m);
		if(closed) return false;
		while(events.empty()){
			new_event.wait(l);
			if(closed) return false;
		}
		bool r = _pop(new_e);
		if( not r ) return false;
		if( e.match(new_e) ){ e = new_e; return true; }
	}
}
bool EventQueue::wait_for_timed(set<Event>& set_e, Event& e, const time_duration& duration, bool& is_timeout){
	Event new_e;
	while(true)
	{
		mutex::scoped_lock l(m);
		if(closed) return false;
		while(events.empty()){
			bool is_not_timeout = new_event.timed_wait(l, duration);
			is_timeout = not is_not_timeout;
			if(closed) return false;
			if(is_timeout) return false;
		}
		bool r = _pop(new_e);
		if( not r ) return false;
		for( set<Event>::const_iterator ee = set_e.begin(); ee!=set_e.end(); ee++ )
		{
			if( ee->match(new_e) ){ e = new_e; return true; }
		}
	}
}
bool EventQueue::wait_for(set<Event>& set_e, Event& e){
	Event new_e;
	while(true)
	{
		mutex::scoped_lock l(m);
		if(closed) return false;
		while(events.empty()){
			new_event.wait(l);
			if(closed) return false;
		}
		bool r = _pop(new_e);
		if( not r ) return false;
		for( set<Event>::const_iterator ee = set_e.begin(); ee!=set_e.end(); ee++ )
		{
			if( ee->match(new_e) ){ e = new_e; return true; }
		}
	}
}
void EventQueue::close_without_remove(){
	mutex::scoped_lock l(m);
	events.clear();
	_close_subscribers();
	parent=0;
	new_event.notify_all();
}
void EventQueue::close(){
	EventQueue* _parent=0;
	{
		mutex::scoped_lock l(m);
		if(closed) return;
		closed = true;
		events.clear();
		_close_subscribers();
		_parent = parent;
		parent=0;
		new_event.notify_all();
	}
	if(_parent){
		_parent->remove(this);
	}
}
void EventQueue::rise(const Event& e){
	if(e.is_template()) return;
	{
		mutex::scoped_lock l(m);
		if(closed) return;
	}
	if(parent==0){
		Event re(e);
		re.stamp_to_now();
		push(re);
		return;
	}
	parent->rise(e);
}

void EventQueue::thread_idle_timer(posix_time::time_duration time){
	try{
		while(not this_thread::interruption_requested()){
			{
				push(idleEvent());
			}
			this_thread::sleep(time);
		}
	}catch (...) { }
}
void EventQueue::run_idle_timer(posix_time::time_duration time){
	mutex::scoped_lock l(m);
	if(idle_timer) return;
	idle_timer = new thread(&EventQueue::thread_idle_timer,this,time);
}
void EventQueue::stop_idle_timer(){
	mutex::scoped_lock l(m);
	if(not idle_timer) return;
	idle_timer->interrupt();
	idle_timer->join();
	delete idle_timer;
	idle_timer=0;
}

bool EventQueue::empty()const{
	mutex::scoped_lock l(m);
	return events.empty();
}
size_t EventQueue::size()const{
	mutex::scoped_lock l(m);
	return events.size();
}
bool EventQueue::is_closed()const{
	mutex::scoped_lock l(m);
	return closed;
}
bool EventQueue::eof()const{ return is_closed(); }


EventQueue& EventQueue::operator>>(Event& e){
	if(not wait_and_pop(e))
		e = Event();
	return *this;
}



}
}

ostream& operator<<(ostream& out, const cognitao::bus::EventQueue& e){
	using namespace cognitao::bus;
	std::string d=" ";
	out<<"{";
	e.m.lock();
	for(list<Event>::const_iterator i=e.events.begin();i!=e.events.end();i++){
		const cognitao::bus::Event& ev = *i;
		out<< d << ev;
		d = " << ";
	}
	e.m.unlock();
	return out<<" }";
}

