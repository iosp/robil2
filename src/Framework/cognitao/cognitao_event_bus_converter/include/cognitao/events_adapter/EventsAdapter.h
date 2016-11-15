/*
 * cognitao_event_bus_converter.h
 *
 *  Created on: Oct 8, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_EVENT_BUS_CONVERTER_EVENTSADAPTER_H_
#define INCLUDE_COGNITAO_EVENT_BUS_CONVERTER_EVENTSADAPTER_H_

#include <cognitao/machine/Events.h>
#include <cognitao/bus/Event.h>
#include <cognitao/bus/StatesMonitor.h>

#include <cognitao/machine/fsm/Fsm.h>
//#include <cognitao/Bt.h>
//#include <cognitao/Ftt.h>

#include <vector>

namespace cognitao{
namespace events_adapter{

class EventsAdapter
{
public:

	virtual
	~EventsAdapter(){}

protected:
	cognitao::bus::Event::scope_t scope_converter( const cognitao::machine::Event::Scope& mev )const
	{
		if( mev == cognitao::machine::Event::GLOBAL ) return cognitao::bus::Event::SCOPE_GLOBAL	;
		if( mev == cognitao::machine::Event::LOCAL  ) return cognitao::bus::Event::SCOPE_DOWN	;
		if( mev == cognitao::machine::Event::SUPER  ) return cognitao::bus::Event::SCOPE_UP		;
		if( mev == cognitao::machine::Event::CHILD  ) return cognitao::bus::Event::SCOPE_CHILD	;
		if( mev == cognitao::machine::Event::PARENT ) return cognitao::bus::Event::SCOPE_PARENT	;
		if( mev == cognitao::machine::Event::SPOT   ) return cognitao::bus::Event::SCOPE_SPOT	;
		return cognitao::bus::Event::SCOPE_GLOBAL;
	}

	cognitao::machine::Event::Scope scope_converter( const cognitao::bus::Event::scope_t& bev )const
	{
		if( bev == cognitao::bus::Event::SCOPE_GLOBAL ) return cognitao::machine::Event::GLOBAL;
		if( bev == cognitao::bus::Event::SCOPE_DOWN   ) return cognitao::machine::Event::LOCAL ;
		if( bev == cognitao::bus::Event::SCOPE_UP     ) return cognitao::machine::Event::SUPER ;
		if( bev == cognitao::bus::Event::SCOPE_CHILD  ) return cognitao::machine::Event::CHILD ;
		if( bev == cognitao::bus::Event::SCOPE_PARENT ) return cognitao::machine::Event::PARENT;
		if( bev == cognitao::bus::Event::SCOPE_SPOT   ) return cognitao::machine::Event::SPOT  ;
		return cognitao::machine::Event::GLOBAL;
	}

	string scope_converter_str( const cognitao::machine::Event::Scope& mev )const
	{
		if( mev == cognitao::machine::Event::GLOBAL ) return ""	;
		if( mev == cognitao::machine::Event::LOCAL  ) return "_"	;
		if( mev == cognitao::machine::Event::SUPER  ) return "^"	;
		if( mev == cognitao::machine::Event::CHILD  ) return "__"	;
		if( mev == cognitao::machine::Event::PARENT ) return "^^"	;
		if( mev == cognitao::machine::Event::SPOT ) return "="	;
		return "";
	}

	string scope_converter_str( const cognitao::bus::Event::scope_t& bev )const
	{
		if( bev == cognitao::bus::Event::SCOPE_GLOBAL ) return ".";
		if( bev == cognitao::bus::Event::SCOPE_DOWN   ) return "_" ;
		if( bev == cognitao::bus::Event::SCOPE_UP     ) return "^" ;
		if( bev == cognitao::bus::Event::SCOPE_CHILD  ) return "__" ;
		if( bev == cognitao::bus::Event::SCOPE_PARENT ) return "^^" ;
		if( bev == cognitao::bus::Event::SCOPE_SPOT ) return "=" ;
		return "";
	}

	cognitao::bus::Event::parameter_t param_converter( const cognitao::machine::Event& mev )const
	{
		cognitao::bus::Event::parameter_t b_params;
		//for(size_t i=0;i<mev.parameters_size();i++) b_params.push_back(mev.parameter(i));
		b_params = mev.parameters();
		return b_params;
	}

	cognitao::machine::Event::Parameters param_converter( const cognitao::bus::Event& bev )const
	{
		cognitao::machine::Event::Parameters m_params;
		//for(size_t i=0;i<bev.parameters_size();i++) m_params.push_back(bev.parameter(i));
		m_params = bev.parameters();
		return m_params;
	}


public:

	virtual
	void on_bus_event( const cognitao::bus::Event& event, std::vector<cognitao::machine::Event>& list )
	{
		std::stringstream mev_stream;
		mev_stream << event.channel();
		mev_stream << "!" << scope_converter_str( scope_converter( event.scope() ) );
		mev_stream << event.name().full_name();
		if(event.parameters().size()>0)
		{
			mev_stream << "(";
			mev_stream << event.parameters();
			mev_stream << ")";
		}
		cognitao::machine::Event mev(mev_stream.str(), cognitao::machine::Context(event.context().full_name()));
		list.push_back(mev);


//		cout <<"[M] ";
//		cout <<"|c "<< event.channel();
//		cout <<"|d "<< "!" <<"|s "<< scope_converter_str( scope_converter( event.scope() ) );
//		cout <<"|n "<< event.name().full_name();
//		if(event.parameters().size()>0)
//		{
//			cout <<"|p "<< "(";
//			cout << event.parameters();
//			cout << ")";
//		}
//		cout<<endl;
	}

	virtual
	void on_machine_event( const cognitao::machine::Event& event, std::vector<cognitao::bus::Event>& list )
	{
		cognitao::bus::Event bev(
			cognitao::bus::Event::name_t( event.name() ),
			cognitao::bus::Event::channel_t( event.channel() ),
			cognitao::bus::Event::context_t( event.context().str() ),
			scope_converter( event.scope() ),
			cognitao::bus::Event::stamp_t(),
			cognitao::bus::Event::source_t(),
			false,
			param_converter( event )
		);
		list.push_back(bev);

		if( event.channel() == cognitao::machine::EventsInterface::machine_report_channel() )
		{
			cognitao::bus::Event::context_t context = HierarchicalName(event.context().str());
			cognitao::bus::Event bev;
			if( boost::starts_with( event.name(), "start" ) ){
				bev = cognitao::monitor::StatesMonitor::begin_event( context );
			}else
			if( boost::starts_with( event.name(), "success") ){
				bev = cognitao::monitor::StatesMonitor::end_event( context , "success" );
			}
			else
			if( boost::starts_with( event.name(), "fail") ){
				bev = cognitao::monitor::StatesMonitor::end_event( context , "failure" );
			}
			else
			if( boost::starts_with( event.name(), "interrupt") ){
				bev = cognitao::monitor::StatesMonitor::end_event( context , "interrupt" );
			}else
			if( boost::starts_with( event.name(), "stop") ){
				bev = cognitao::monitor::StatesMonitor::end_event( context , "unknown" );
			}

			list.push_back(bev);
		}
	}
};

}
}


#endif /* INCLUDE_COGNITAO_EVENT_BUS_CONVERTER_EVENTSADAPTER_H_ */
