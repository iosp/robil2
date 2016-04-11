
#include <iostream>
#include <ros/ros.h>

// need
#include <decision_making/SynchCout.h>
#include "ComponentStates.h"

// OLD B
//			#include <decision_making/BT.h>
//			#include <decision_making/FSM.h>
//			#include <decision_making/ROSTask.h>
//			#include <decision_making/DecisionMaking.h>
//			#include <decision_making/DebugModeTracker.hpp>
// OLD E





using namespace std;
// OLD B
//using namespace decision_making;

//class Params: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	Params(ComponentMain* comp):comp(comp){}
//	std::string str()const{return "";}
//};
// OLD E





void process_machine(cognitao::machine::Machine & machine, Processor & processor, ComponentMain& component){
	while(processor.empty() == false){
		cognitao::machine::Event e_poped = processor.pop();
		cout << "       PROCESS: " << e_poped.str() << endl;;
		cognitao::machine::Events p_events;
		machine = machine->process(e_poped, p_events);
		processor.insert( p_events );

		static const cognitao::machine::Event event_about_entry_to_state( "task_report?enter" );
		if( event_about_entry_to_state.matches(e_poped) )
		{
			size_t context_size = e_poped.context().size();
			string current_event_context = e_poped.context().str();
			if (context_size > 1){
				std::string current_task = e_poped.context()[context_size-2];
				ROS_WARN_STREAM (" Current task: " << current_task);
				ROS_INFO_STREAM (" Current event context: " << current_event_context);
				if (current_task == "init") {
					cognitao::bus::Event ev_bus_event (cognitao::bus::Event::name_t("pp/EndOfInit"),
													   cognitao::bus::Event::channel_t(""),
													   cognitao::bus::Event::context_t(current_event_context));
					processor.bus_events << ev_bus_event;
				}
				if (current_task == "ready") {
					component.resume_navigation();
					component.rise_taskStarted();
					ROS_WARN_STREAM ("Navigation is resumed");
				}
				if (current_task == "standby") {
					component.cancel_navigation();
					component.rise_taskPaused();
					ROS_WARN_STREAM ("Navigation is canceled");
				}
			}
		}
	}
}








// OLD B

//FSM(pp_WORK)
//{
//	FSM_STATES
//	{
//		STANDBY,
//		READY
//	}
//	FSM_START(READY);
//	FSM_BGN
//	{
//		FSM_STATE(STANDBY)
//		{
//			FSM_CALL_TASK(STANDBY);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/pp/Resume", FSM_NEXT(READY));
//			}
//		}
//		FSM_STATE(READY)
//		{
//			FSM_CALL_TASK(READY);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/pp/Standby", FSM_NEXT(STANDBY));
//			}
//		}
//
//	}
//	FSM_END
//}
//FSM(pp_ON)
//{
//	FSM_STATES
//	{
//		INIT,
//		WORK
//	}
//	FSM_START(INIT);
//	FSM_BGN
//	{
//		FSM_STATE(INIT)
//		{
//			FSM_CALL_TASK(INIT);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("INIT/EndOfInit", FSM_NEXT(WORK));
//			}
//		}
//		FSM_STATE(WORK)
//		{
//			FSM_CALL_FSM(pp_WORK)
//			FSM_TRANSITIONS{}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(pp)
//{
//	FSM_STATES
//	{
//		OFF,
//		ON
//	}
//	FSM_START(ON);
//	FSM_BGN
//	{
//		FSM_STATE(OFF)
//		{
//			FSM_CALL_TASK(OFF);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
//				FSM_ON_EVENT("/pp/Activation", FSM_NEXT(ON));
//			}
//		}
//		FSM_STATE(ON)
//		{
//			FSM_CALL_FSM(pp_ON)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
//				FSM_ON_EVENT("/pp/Shutdown", FSM_NEXT(OFF));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
//	PAUSE(10000);
//	//diagnostic_msgs::DiagnosticStatus status;
//	//COMPONENT->publishDiagnostic(status);
//	return TaskResult::SUCCESS();
//}
//TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
//	PAUSE(1000);
//	events.raiseEvent(Event("EndOfInit",context));
//	return TaskResult::SUCCESS();
//}
//TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
//	COMPONENT->resume_navigation();
//	COMPONENT->rise_taskStarted();
//	return TaskResult::SUCCESS();
//}
//TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
//	COMPONENT->cancel_navigation();
//	COMPONENT->rise_taskPaused();
//	return TaskResult::SUCCESS();
//}
// OLD E


void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000, "/robil/event_bus/events");

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream	<< "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl
								<< "<tao>" << endl
								<< "	<machines>" << endl
								<< "		<machine file=\"${rospack:pp}/src/xml/pp.xml\"/>" << endl
								<< "		<root>pp</root>" << endl
								<< "	</machines>" << endl
								<< "</tao>" << endl;


	cognitao::machine::Context context ("path_planer"); // TODO do  need some context?
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::MachinesCollection machines;
	try{
		machines = parser.parse(mission_description_stream, context.str());
	} catch(const cognitao::io::parser::ParsingError& error){
		std::cerr <<"ParsingError:"<<endl<< error.message <<endl;
		return;
	}

	cognitao::io::compiler::Compiler compiler;
	Processor processor( events );
	compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::fsm::FsmBuilder(processor) ) );
	compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::ftt::FttBuilder(processor) ) );

	cognitao::io::compiler::CompilationObjectsCollector collector;
	cognitao::io::compiler::CompiledMachine ready_machine;
	try {
		ready_machine = compiler.compile( machines, collector );
	} catch(const cognitao::io::compiler::CompilerError& error){
		std::cerr <<"CompilerError:"<<endl<< error.message <<endl;
		return;
	}

	cout << endl << endl;
	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine = ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_machine (current_machine, processor, component);

	time_duration max_wait_duration (0, 0, 5, 0);
	bool is_timeout = false;
	cognitao::bus::Event event;
	while(events.wait_and_pop_timed(event, max_wait_duration, is_timeout) or ros::ok())
	{
		if (is_timeout){
//			cout << "event bus timeout" << endl;
			continue;
		}
		cout << "GET: " << event << endl;
		if (event.context().str().find(context.str()) != 0) {
			cout << "\033[1;31m SKIP event from other node \033[0m\n";
			continue;
		}
		processor.send_no_pub (event);
		process_machine (current_machine, processor, component);
	}







// OLD B
//	ros_decision_making_init(argc, argv);
//	RosEventQueue events;
//	component.set_events(&events);
//	CallContext context;
//	context.createParameters(new Params(&component));
//	//events.async_spin();
//	LocalTasks::registration("OFF",state_OFF);
//	LocalTasks::registration("INIT",state_INIT);
//	LocalTasks::registration("READY",state_READY);
//	LocalTasks::registration("STANDBY",state_STANDBY);
//
//	ROS_INFO("Starting pp (PathPlanner)...");
//	Fsmpp(&context, &events);
//	component.set_events(NULL);
// OLD E
	return;
}


