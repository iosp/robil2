/*
 * cognitao_v2.cpp
 *
 *  Created on: Jul 24, 2016
 *      Author: misha
 */

#include <cognitao_v2/cognitao_v2.h>

std::vector<cognitao::machine::Event> events_bus_to_internal(
		const cognitao::bus::Event& original) {
	std::vector<cognitao::machine::Event> mresult;
	mresult.clear();
	cognitao::events_adapter::FsmEventsAdapter fsm_adapter;
	cognitao::events_adapter::FttEventsAdapter ftt_adapter;
	cognitao::events_adapter::EventsAdapter general_adapter;
	general_adapter.on_bus_event(original, mresult);
	fsm_adapter.on_bus_event(original, mresult);
	ftt_adapter.on_bus_event(original, mresult);
	return mresult;
}

std::vector<cognitao::bus::Event> internal_event_to_bus(
		const cognitao::machine::Event& original) {
	std::vector<cognitao::bus::Event> bresult;
	bresult.clear();
	cognitao::events_adapter::FsmEventsAdapter fsm_adapter;
	cognitao::events_adapter::FttEventsAdapter ftt_adapter;
	cognitao::events_adapter::EventsAdapter general_adapter;
	general_adapter.on_machine_event(original, bresult);
	fsm_adapter.on_machine_event(original, bresult);
	ftt_adapter.on_machine_event(original, bresult);
	return bresult;
}
;

Processor::Processor(cognitao::bus::EventRiser& events) :
		bus_events(events) {
}
;

bool Processor::empty() const {
	return queue.empty();
}

cognitao::machine::Event Processor::pop() {
	cognitao::machine::Event e = queue.front();
	queue.pop_front();
	return e;
}

void Processor::insert(cognitao::machine::Events& events) {
	BOOST_FOREACH(const cognitao::machine::Event& e, events.events()) {
		send(e);
	}
}

void Processor::insert_no_pub(cognitao::machine::Events& events) {
	BOOST_FOREACH(const cognitao::machine::Event& e, events.events()) {
		send_no_pub(e);
	}
}

void Processor::send(const cognitao::machine::Event& original) {
	send_bus_event(original);
}

void Processor::send_no_pub(const cognitao::machine::Event& original) {
	queue.push_back(original); // add events throw RosEventQueue
//	cout << "           ADD: " << original << endl;
}

void Processor::send_no_pub(const cognitao::bus::Event & event) {
//	cout << "       CONVERT: " << event << endl;
	std::vector<cognitao::machine::Event> internal_events_array =
			events_bus_to_internal(event);
	BOOST_FOREACH ( const cognitao::machine::Event& e, internal_events_array ) {
		send_no_pub(e);
	}
}

void Processor::send_bus_event(const cognitao::machine::Event& e) {
	cout << "       CONVERT: " << e << endl;
	std::vector<cognitao::bus::Event> bus_events_array = internal_event_to_bus(
			e);
	BOOST_FOREACH( const cognitao::bus::Event& bus_e, bus_events_array ) {
		bus_events << bus_e;
		cout << "           PUB: " << bus_e << endl;
	}
}
