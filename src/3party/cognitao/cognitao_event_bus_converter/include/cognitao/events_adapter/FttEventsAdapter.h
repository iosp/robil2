/*
 * cognitao_event_bus_converter.h
 *
 *  Created on: Oct 8, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_FTT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_
#define INCLUDE_COGNITAO_FTT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_

#include <cognitao/events_adapter/EventsAdapter.h>

namespace cognitao{
namespace events_adapter{

class FttEventsAdapter: public EventsAdapter
{
public:

	virtual
	~FttEventsAdapter(){}

	virtual
	void on_bus_event( const cognitao::bus::Event& event, std::vector<cognitao::machine::Event>& list )
	{

	}

	virtual
	void on_machine_event( const cognitao::machine::Event& event, std::vector<cognitao::bus::Event>& list )
	{
		if( event.channel() == "task_report" )
		{
			cognitao::bus::Event::context_t context = HierarchicalName(event.context().str());
			cognitao::bus::Event bev;
			if( boost::starts_with( event.name(), "enter" ) ){
				bev = cognitao::monitor::StatesMonitor::begin_event( context );
			}else
			if( boost::starts_with( event.name(), "exit") ){
				bev = cognitao::monitor::StatesMonitor::end_event( context , "unknown" );
			}else
			return;

			bev.parameters( event.parameters() );
			list.push_back(bev);
		}
	}
};



}
}


#endif /* INCLUDE_COGNITAO_FTT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_ */
