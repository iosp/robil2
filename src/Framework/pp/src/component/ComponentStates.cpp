
#include <iostream>
#include <ros/ros.h>

// need
#include <decision_making/SynchCout.h>
#include "ComponentStates.h"

// don't need
			#include <decision_making/BT.h>
			#include <decision_making/FSM.h>
			#include <decision_making/ROSTask.h>
			#include <decision_making/DecisionMaking.h>
			#include <decision_making/DebugModeTracker.hpp>

// for new cognitao
#pragma push_macro("cout")
#undef cout
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>
#include <cognitao/io/compiler/fsm/FsmBuilder.h>
#include <cognitao/io/compiler/ftt/FttBuilder.h>
#include <cognitao/bus/ros_events_bus.h>
#pragma pop_macro("cout")





using namespace std;
// don't need
			using namespace decision_making;



class Params: public CallContextParameters{
public:
	ComponentMain* comp;
	Params(ComponentMain* comp):comp(comp){}
	std::string str()const{return "";}
};




// for cognitao

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
		BOOST_FOREACH(const cognitao::machine::Event& e, events.events())
		{
			send( e );
		}
	}

	virtual
	void on_match(const cognitao::machine::Event& original, const cognitao::machine::Event& stranger){};

	virtual
	void send(const cognitao::machine::Event& original){
		queue.push_back(original);
		send_bus_event( original );
	}

	void send_bus_event( const cognitao::machine::Event& e ){ // TODO update according to current tasks
		static const cognitao::machine::Event join_event ( "join" );
		static const cognitao::machine::Event intentional_leave_event ( "intentional_leave" );
		static const cognitao::machine::Event unintentional_permanent_leave_event ( "unintentional_permanent_leave" );
		static const cognitao::machine::Event return_event ( "return" );
		static const cognitao::machine::Event temporal_leave_event ( "temporal_leave" );

		// if event comes from event bus (generated on the base of cognitao::bus::Event), we skip it
		// if event comes from internal machine (directly cognitao::machine::Event), we process it and publish correspondent event to event bus
		// we need this check to avoid endless loop
		if (e.context().empty()){
			return;
		}

		Event ev_bus (Event::name_t("join"), Event::channel_t("synchronization"), Event::context_t(agent_name) );
		bus_events << ev_bus;
		ROS_WARN_STREAM ("				PUB: " << ev_bus);

	}


	virtual
	void on_private(const cognitao::machine::Event& original){}
};



bool events_bus_to_internal_event (const cognitao::bus::Event & input_event, cognitao::machine::Event & output_event, std::string & agent){

	std::vector<std::string> parts;
	parts = input_event.context().split();
	if (parts.size() == 0){
		return false;
	}

	if( input_event.channel()=="states"){
		if (parts.front() == "agent"){
			if (cognitao::monitor::StatesMonitor::is_state_begin(input_event)){
				agent = parts[1];
				output_event = cognitao::machine::Event ("!added");
			} else if (cognitao::monitor::StatesMonitor::is_state_end(input_event)){
				agent = parts[1];
				output_event = cognitao::machine::Event ("!dead");
			} else return false;
		}else if (  cognitao::bus::Event::context_t("/self/family").is_prefix_of(input_event.context()) ){
			agent = agent_name;
			if (cognitao::monitor::StatesMonitor::is_state_begin(input_event)){
				output_event = cognitao::machine::Event ("!has_family_info");
			} else {
				output_event = cognitao::machine::Event ("!no_family_info");
			}
		} else {
			return false;
		}
		return true;
	} else if (input_event.channel()=="synchronization"){
		agent = parts.front();
		std::stringstream ss;
		ss << "!" <<input_event.name();
		output_event = cognitao::machine::Event (ss.str());
		return true;
	} else if (input_event.name()=="command_to_leave"){
		agent = agent_name;
		std::stringstream ss;
		ss << "!" <<input_event.name();
		output_event = cognitao::machine::Event (ss.str());
		return true;
	} else {
		return false;
	}
}



void on_new_event (const cognitao::bus::Event & event){
//	/agent/R2/states./begin    					- added
//  /agent/R2/states./end/is_dead 				- dead
//	/self/family/FAM2/states./begin				- has_family_info
//	/self/family/FAM1/states./end/success		- no_family_info

//	ROS_INFO_STREAM ("Event bus e: " << event);
	cognitao::machine::Event check_event("null");
	if (not events_bus_to_internal_event (event, check_event, agent_name)){
		ROS_INFO_STREAM ("	SKIP     " << event);
		return;
	}
	ROS_INFO_STREAM ("	CONVERT  " << event << " --> " << check_event << "  [" << agent_name << "]");


	MachinePtr& machine = machines_map[agent_name];
	if( not machine)
	{
		machine.reset( new Machine(agent_name, events, false) );
	}
	machine->on_new_event( check_event );

}

void process_machine(cognitao::machine::Machine & machine, Processor & processor){
	while(processor.empty() == false){
		cognitao::machine::Event e = processor.pop();
		std::string current_event = e.str();
		ROS_INFO_STREAM ("Current event: " << current_event);
		cognitao::machine::Events p_events;
		machine = machine->process(e, p_events);
		processor.insert( p_events );
	}
}














FSM(pp_WORK)
{
	FSM_STATES
	{
		STANDBY,
		READY
	}
	FSM_START(READY);
	FSM_BGN
	{
		FSM_STATE(STANDBY)
		{
			FSM_CALL_TASK(STANDBY);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/pp/Resume", FSM_NEXT(READY));
			}
		}
		FSM_STATE(READY)
		{
			FSM_CALL_TASK(READY);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/pp/Standby", FSM_NEXT(STANDBY));
			}
		}

	}
	FSM_END
}
FSM(pp_ON)
{
	FSM_STATES
	{
		INIT,
		WORK
	}
	FSM_START(INIT);
	FSM_BGN
	{
		FSM_STATE(INIT)
		{
			FSM_CALL_TASK(INIT);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("INIT/EndOfInit", FSM_NEXT(WORK));
			}
		}
		FSM_STATE(WORK)
		{
			FSM_CALL_FSM(pp_WORK)
			FSM_TRANSITIONS{}
		}

	}
	FSM_END
}

FSM(pp)
{
	FSM_STATES
	{
		OFF,
		ON
	}
	FSM_START(ON);
	FSM_BGN
	{
		FSM_STATE(OFF)
		{
			FSM_CALL_TASK(OFF);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
				FSM_ON_EVENT("/pp/Activation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(pp_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
				FSM_ON_EVENT("/pp/Shutdown", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
	PAUSE(10000);
	//diagnostic_msgs::DiagnosticStatus status;
	//COMPONENT->publishDiagnostic(status);
	return TaskResult::SUCCESS();
}
TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
	PAUSE(1000);
	events.raiseEvent(Event("EndOfInit",context));
	return TaskResult::SUCCESS();
}
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	COMPONENT->resume_navigation();
	COMPONENT->rise_taskStarted();
	return TaskResult::SUCCESS();
}
TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
	COMPONENT->cancel_navigation();
	COMPONENT->rise_taskPaused();
	return TaskResult::SUCCESS();
}

void runComponent(int argc, char** argv, ComponentMain& component){

	// cognitao
	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000, "/pp/event_bus/events");


	std::istream& mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <tao>	<machines> <machine include = \"${rospack:pp}/src/xml/pp.xml\">	</machines>	</tao>";
	cognitao::machine::Context context ("/plan/pp");
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

	cognitao::bus::Event event;
	while( events.wait_and_pop(event) )
	{
		on_new_event( event );
		process_machine (current_machine, processor);
	}





	ros_decision_making_init(argc, argv);
	RosEventQueue events;
	component.set_events(&events);
	CallContext context;
	context.createParameters(new Params(&component));
	//events.async_spin();
	LocalTasks::registration("OFF",state_OFF);
	LocalTasks::registration("INIT",state_INIT);
	LocalTasks::registration("READY",state_READY);
	LocalTasks::registration("STANDBY",state_STANDBY);

	ROS_INFO("Starting pp (PathPlanner)...");
	Fsmpp(&context, &events);
	component.set_events(NULL);
}


