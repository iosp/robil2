/*
 * MachineEventsInterface.h
 *
 *  Created on: Nov 25, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_MACHINEEVENTSINTERFACE_H_
#define INCLUDE_COGNITAO_MACHINEEVENTSINTERFACE_H_

#include "Events.h"

namespace cognitao{
namespace machine{

	class EventsInterface{
	public:

		static string machine_command_channel(){ return "machine_control"; }
		static string machine_report_channel() { return "machine_report" ; }

		Events start_report;
		Events success_report;
		Events fail_report;
		Events interrupt_report;
		Events stop_report;

		Events private_success_report;
		Events private_fail_report;

		Events interrupt_command;
		Events success_command;
		Events fail_command;

		EventsInterface()
		{

			start_report				.add( machine_report_start() );
			stop_report					.add( machine_report_stop() );
			interrupt_report			.add( machine_report_interrupt() );
			success_report				.add( machine_report_success() );
			fail_report					.add( machine_report_fail() );

			private_success_report		.add( machine_command_success(false) );
			private_fail_report			.add( machine_command_fail(false) );

			interrupt_command	.add( machine_command_interrupt() );
			success_command		.add( machine_command_success() );
			fail_command		.add( machine_command_fail() );
		}

		static const Event machine_report_start(bool out=true){ return Event(machine_report_channel()+(out?"!":"?")+"start"); }
		static const Event machine_report_fail(bool out=true){ return Event(machine_report_channel()+(out?"!":"?")+"fail"); }
		static const Event machine_report_success(bool out=true){ return Event(machine_report_channel()+(out?"!":"?")+"success"); }
		static const Event machine_report_interrupt(bool out=true){ return Event(machine_report_channel()+(out?"!":"?")+"interrupt"); }
		static const Event machine_report_stop(bool out=true){ return Event(machine_report_channel()+(out?"!":"?")+"stop"); }

		static const Event machine_command_interrupt(bool in=true){ return Event(machine_command_channel()+(in?"?^":"!_")+"interrupt"); }
		static const Event machine_command_success(bool in=true){ return Event(machine_command_channel()+(in?"?__":"!^^")+"success"); }
		static const Event machine_command_fail(bool in=true){ return Event(machine_command_channel()+(in?"?__":"!^^")+"fail"); }

	};

}
}

#endif /* INCLUDE_COGNITAO_MACHINEEVENTSINTERFACE_H_ */
