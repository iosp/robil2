/*
 * cognitao_event_bus_converter.h
 *
 *  Created on: Oct 8, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_BT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_
#define INCLUDE_COGNITAO_BT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_

#include <cognitao/events_adapter/EventsAdapter.h>

namespace cognitao{
namespace events_adapter{

class BtEventsAdapter: public EventsAdapter
{
public:

	virtual
	~BtEventsAdapter(){}



	virtual
	void on_bus_event( const cognitao::bus::Event& event, std::vector<cognitao::machine::Event>& list )
	{

	}

	virtual
	void on_machine_event( const cognitao::machine::Event& event, std::vector<cognitao::bus::Event>& list )
	{
		if( event.channel() == "behavior_report" )
		{
			cognitao::bus::Event::context_t context = HierarchicalName(event.context().str());
			cognitao::bus::Event bev;
#define TRANS_BGN( N ) \
			if( boost::starts_with( event.name(), N ) ){ \
				bev = cognitao::monitor::StatesMonitor::begin_event( context ); \
			}else

#define TRANS_END( N, R ) \
			if( boost::starts_with( event.name(), N ) ){ \
				bev = cognitao::monitor::StatesMonitor::end_event( context , R ); \
			}else

			/*
			 	behavior!fault
				behavior!success
				behavior!next
			 */

//			TRANS_BGN("started")
//			TRANS_END("failt")

			struct NOT_COMPLITED{};
			throw NOT_COMPLITED();
			return;

#undef TRANS_BGN
#undef TRANS_END

			list.push_back(bev);
		}
	}
};



}
}


#endif /* INCLUDE_COGNITAO_BT_EVENT_BUS_CONVERTER_COGNITAO_EVENT_BUS_CONVERTER_H_ */
