/*
 * cognitao_v2.h
 *
 *  Created on: Jul 24, 2016
 *      Author: assaf
 */

#ifndef FRAMEWORK_IEDSIM_SRC_COGNITAO_V2_COGNITAO_V2_H_
#define FRAMEWORK_IEDSIM_SRC_COGNITAO_V2_COGNITAO_V2_H_

#pragma push_macro("cout")
#undef cout
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>
#include <cognitao/io/compiler/fsm/FsmBuilder.h>
#include <cognitao/io/compiler/ftt/FttBuilder.h>
#include <cognitao/bus/ros_events_bus.h>
#include <cognitao/events_adapter/FsmEventsAdapter.h>
#include <cognitao/events_adapter/FttEventsAdapter.h>
#pragma pop_macro("cout")

typedef list<cognitao::machine::Event> LocalEventsQueue;
std::vector<cognitao::machine::Event> events_bus_to_internal(
		const cognitao::bus::Event& original);
std::vector<cognitao::bus::Event> internal_event_to_bus(
		const cognitao::machine::Event& original);

class Processor: public cognitao::machine::EventProcessor {
public:
	LocalEventsQueue queue;
	cognitao::bus::EventRiser& bus_events;

	Processor(cognitao::bus::EventRiser& events);
	virtual ~Processor() {
	}
	;
	bool empty() const;
	cognitao::machine::Event pop();
	void insert(cognitao::machine::Events& events);
	void insert_no_pub(cognitao::machine::Events& events);
	virtual void on_match(const cognitao::machine::Event& original,
			const cognitao::machine::Event& stranger) {
	}
	;
	virtual void send(const cognitao::machine::Event& original);
	void send_no_pub(const cognitao::machine::Event& original);
	void send_no_pub(const cognitao::bus::Event & event);
	void send_bus_event(const cognitao::machine::Event& e);
	virtual void on_private(const cognitao::machine::Event& original) {
	}
	;
};

#endif /* FRAMEWORK_IEDSIM_SRC_COGNITAO_V2_COGNITAO_V2_H_ */
