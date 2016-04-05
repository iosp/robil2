
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





using namespace std;
// OLD B
//			using namespace decision_making;

//class Params: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	Params(ComponentMain* comp):comp(comp){}
//	std::string str()const{return "";}
//};
// OLD E


std::vector<cognitao::machine::Event> events_bus_to_internal (const cognitao::bus::Event& original){
	std::vector<cognitao::machine::Event> mresult;
	cognitao::events_adapter::FsmEventsAdapter fsm_adapter;
    cognitao::events_adapter::FttEventsAdapter ftt_adapter;
	cognitao::events_adapter::EventsAdapter general_adapter;
	general_adapter.on_bus_event( original, mresult );
	fsm_adapter.on_bus_event( original, mresult );
    ftt_adapter.on_bus_event( original, mresult );
    return mresult;
}

std::vector<cognitao::bus::Event> internal_event_to_bus(const cognitao::machine::Event& original){
	std::vector<cognitao::bus::Event> bresult;
	bresult.clear();
	cognitao::events_adapter::FsmEventsAdapter fsm_adapter;
	cognitao::events_adapter::FttEventsAdapter ftt_adapter;
	cognitao::events_adapter::EventsAdapter general_adapter;
	general_adapter.on_machine_event( original, bresult );
	fsm_adapter.on_machine_event( original, bresult );
	ftt_adapter.on_machine_event( original, bresult );
	return bresult;
};

typedef list<cognitao::machine::Event> LocalEventsQueue;
class Processor:public cognitao::machine::EventProcessor {
public:
	LocalEventsQueue queue;
	cognitao::bus::EventRiser& bus_events;

	Processor (cognitao::bus::EventRiser& events):bus_events(events){}

	virtual ~Processor(){};

	bool empty()const{
		return queue.empty();
	}

	cognitao::machine::Event pop(){
		cognitao::machine::Event e = queue.front();
		queue.pop_front();
		return e;
	}

	void insert( cognitao::machine::Events& events ){
		cout << "insert called with " << events.events().size() << endl;
		BOOST_FOREACH(const cognitao::machine::Event& e, events.events())
		{
			send(e);
			send_bus_event(e);
		}
	}

	virtual
	void on_match(const cognitao::machine::Event& original, const cognitao::machine::Event& stranger){};

	virtual
	void send(const cognitao::machine::Event& original){
		cout << "send called with " << original.str() << endl;
		queue.push_back(original); // add events throw RosEventQueue
//		send_bus_event(original);
	}

	void send_bus_event( const cognitao::machine::Event& e ){
		cout << "send_bus_event called with " << e.str() << endl;
		std::vector<cognitao::bus::Event> bus_events_array = internal_event_to_bus(e);
		BOOST_FOREACH( const cognitao::bus::Event& bus_e, bus_events_array )
		{
			cout << "		PUB: " << bus_e << endl;
			bus_events << bus_e;
		}
	}

	virtual
	void on_private(const cognitao::machine::Event& original){}
};


void on_new_event (const cognitao::bus::Event & event, Processor & processor){
	cout << "on_new_event is called with " << event << endl;
	std::vector<cognitao::machine::Event> internal_events_array = events_bus_to_internal (event);
	BOOST_FOREACH ( const cognitao::machine::Event& e, internal_events_array )
	{
		processor.send(e);
	}
}

void process_machine(cognitao::machine::Machine & machine, Processor & processor){
	cout << "process_machine is called with " << endl;
	while(processor.empty() == false){
		cognitao::machine::Event e = processor.pop();
		std::string current_event = e.str();
		ROS_INFO_STREAM ("PROCESS EVENT: " << current_event);
		cognitao::machine::Events p_events;
		machine = machine->process(e, p_events);
		processor.insert( p_events );

		static const cognitao::machine::Event event_about_entry_to_state( "state_report?enter" );
		if( event_about_entry_to_state.matches(e) )
		{
			std::string current_state = e.context().tail();
			ROS_WARN_STREAM (" Current state" << current_state);
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

bool load_from_file (std::string full_filename, std::stringstream & xml_file){
	string line;
	ifstream myfile (full_filename.c_str());
	if (myfile.is_open()){
		while ( getline (myfile,line) ){
			xml_file << line << '\n';
		}
		myfile.close();
	} else {
		ROS_ERROR_STREAM ("Error while openning the file " << myfile);
		return false;
	}
	return true;
}




void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000, "/pp/event_bus/events");


	std::stringstream mission_description_stream;
//	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <tao>	<machines> <machine include = \"${rospack:pp}/src/xml/pp.xml\">	</machines>	</tao>";

	if (not load_from_file("/home/misha/workspaces/robil_workspace/robil2/src/Framework/pp/src/xml/pp.xml", mission_description_stream)){
		return;
	} // TODO change to normal loading of xml file

	cognitao::machine::Context context;
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::MachinesCollection machines = parser.parse(mission_description_stream, context.str());

	cognitao::io::compiler::Compiler compiler;
	Processor processor( events );
	compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::fsm::FsmBuilder(processor) ) );
	compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::ftt::FttBuilder(processor) ) );

	cognitao::io::compiler::CompilationObjectsCollector collector;
	cognitao::io::compiler::CompiledMachine ready_machine = compiler.compile( machines, collector );

	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine = ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_machine (current_machine, processor);

	cognitao::bus::Event event;
	while( events.wait_and_pop(event) and ros::ok())
	{
		cout << "EVENT_BUS GET: " << event << endl;
		on_new_event(event, processor);
		process_machine (current_machine, processor);
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
	cout << "Exit from runComponent" << endl;
	return;
}


