#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
//#include <decision_making/DebugModeTracker.hpp>


using namespace std;
//using namespace decision_making;
#include "ComponentStates.h"
#include "AblManager.h"
#include "EventTranslator.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define EVENT(X) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(context))
#define RAISE(X) processor_ptr->bus_events << EVENT(X)

//class Params: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	Params(ComponentMain* comp):comp(comp){}
//	std::string str()const{return "";}
//};

class SysTask {
protected:
	ComponentMain * comp_ptr;
	boost::thread run_thread;
	Processor * processor_ptr;
	std::string context;

public:
	SysTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : comp_ptr(comp),processor_ptr(processor),context(context) {
		run_thread = boost::thread(boost::bind(&SysTask::run, this));
	}

	virtual void run() = 0;

	void pause(int millisecs) {
		int msI = (millisecs / 100), msR = (millisecs % 100);
		for (int si = 0; si < msI and not comp_ptr->events()->is_closed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->events()->is_closed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void offTask() {
		pause(10000);
	}

	virtual ~SysTask() {
		run_thread.interrupt();
		run_thread.join();
		comp_ptr = NULL;
		processor_ptr = NULL;
	}
};

class SysInitTask : public SysTask {
public:
	SysInitTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : SysTask(comp,processor,current_context) {
		run_thread = boost::thread(boost::bind(&SysInitTask::run, this));
	}

	void run() {
		//pause(10000);
		RAISE("/EndOfCoreSystemInit");
	}
};

class SysReadyTask : public SysTask {
protected:
	AblManager * abl_m;

public:
	SysReadyTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : SysTask(comp,processor,current_context) {
		abl_m = new AblManager(comp_ptr);
		run_thread = boost::thread(boost::bind(&SysReadyTask::run, this));
	}

	void run() {
		pause(10000);
		abl_m->listen(comp_ptr->events());
		EventTranslator(comp_ptr, comp_ptr->events());
	}

	void ~SysReadyTask() {
		delete abl_m;
	}
};

class SysEmergencyTask : public SysTask {
public:
	SysEmergencyTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : SysTask(comp,processor,current_context) {
		run_thread = boost::thread(boost::bind(&SysEmergencyTask::run, this));
	}

	void run() {
		pause(10000);
	}
};

//FSM(smme_ON)
//{
//	FSM_STATES
//	{
//		Init,
//		Ready,
//		Emergency
//	}
//	FSM_START(Init);
//	FSM_BGN
//	{
//		FSM_STATE(Init)
//		{
//			FSM_CALL_TASK(SYS_INIT)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/EndOfCoreSystemInit", FSM_NEXT(Ready));
//			}
//		}
//		FSM_STATE(Ready)
//		{
//			FSM_CALL_TASK(SYS_READY)
//			FSM_CALL_TASK(ABL)
//			FSM_CALL_TASK(EventTranslator)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/EStopCommand", FSM_NEXT(Emergency));
//			}
//		}
//		FSM_STATE(Emergency)
//		{
//			FSM_CALL_TASK(SYS_EMERGENCY)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/ClearEStopCommand", FSM_NEXT(Ready));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(smme)
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
//			FSM_CALL_TASK(SYS_OFF)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/SystemActivation", FSM_NEXT(ON));
//			}
//		}
//		FSM_STATE(ON)
//		{
//			FSM_CALL_FSM(smme_ON)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/PowerOff", FSM_NEXT(OFF));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//namespace {
//
//TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
//	PAUSE(10000);
//	//diagnostic_msgs::DiagnosticStatus status;
//	//COMPONENT->publishDiagnostic(status);
//	return TaskResult::SUCCESS();
//}
//TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
//	//PAUSE(10000);
//	events.raiseEvent(Event("/EndOfCoreSystemInit",context));
//	return TaskResult::SUCCESS();
//}
//TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
//	PAUSE(10000);
//	return TaskResult::SUCCESS();
//}
//TaskResult state_EMERGENCY(string id, const CallContext& context, EventQueue& events){
//	PAUSE(10000);
//	return TaskResult::SUCCESS();
//}
//TaskResult tsk_ABL(string id, const CallContext& context, EventQueue& events){
//	AblManager abl(COMPONENT);
//	abl.listen(&events);
//	return TaskResult::SUCCESS();
//}
//TaskResult tsk_EventTranslator(string id, const CallContext& context, EventQueue& events){
//	EventTranslator(COMPONENT, &events);
//	return TaskResult::SUCCESS();
//}
//
//}

SysTask* systask_ptr;

void process_machine(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
		cout << "       PROCESS: " << e_poped.str() << endl;
		;
		cognitao::machine::Events p_events;
		machine = machine->process(e_poped, p_events);
		processor.insert(p_events);

		static const cognitao::machine::Event event_about_entry_to_state(
				"task_report?enter");
		if (event_about_entry_to_state.matches(e_poped)) {
			size_t context_size = e_poped.context().size();
			string current_event_context = e_poped.context().str();
			if (context_size > 1) {
				std::string current_task = e_poped.context()[context_size - 2];
				ROS_WARN_STREAM(" Current state: " << current_task);
				ROS_INFO_STREAM(
						" Current event context: " << current_event_context);
				if (current_task == "off")
					systask_ptr->offTask();
				DELETE(systask_ptr);
				if (current_task == "init")
					systask_ptr = new SysInitTask(&component, &processor,
							current_event_context);
				if (current_task == "ready")
					systask_ptr = new SysReadyTask(&component, &processor,
							current_event_context);
				if (current_task == "emergency")
					systask_ptr = new SysEmergencyTask(&component, &processor,
							current_event_context);
				cout << "address: " << systask_ptr << endl;
			}
		}
	}
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000,
			"/robil/event_bus/events");

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:smme}/src/xml/smme.xml\"/>"
			<< endl << "		<root>smme</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("smme"); // TODO do  need some context?
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::MachinesCollection machines;
	try {
		machines = parser.parse(mission_description_stream, context.str());
	} catch (const cognitao::io::parser::ParsingError& error) {
		std::cerr << "ParsingError:" << endl << error.message << endl;
		return;
	}

	cognitao::io::compiler::Compiler compiler;
	Processor processor(events);
	compiler.add_builder(
			cognitao::io::compiler::MachineBuilder::Ptr(
					new cognitao::io::compiler::fsm::FsmBuilder(processor)));
	compiler.add_builder(
			cognitao::io::compiler::MachineBuilder::Ptr(
					new cognitao::io::compiler::ftt::FttBuilder(processor)));

	cognitao::io::compiler::CompilationObjectsCollector collector;
	cognitao::io::compiler::CompiledMachine ready_machine;
	try {
		ready_machine = compiler.compile(machines, collector);
	} catch (const cognitao::io::compiler::CompilerError& error) {
		std::cerr << "CompilerError:" << endl << error.message << endl;
		return;
	}

	cout << endl << endl;
	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine =
			ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_machine(current_machine, processor, component);

	time_duration max_wait_duration(0, 0, 5, 0);
	bool is_timeout = false;
	cognitao::bus::Event event;
	while (events.wait_and_pop_timed(event, max_wait_duration, is_timeout)
			or ros::ok()) {
		if (is_timeout) {
//			cout << "event bus timeout" << endl;
			continue;
		}
		cout << "GET: " << event << endl;
//		if (event.context().str().find(context.str()) != 0) {
//			cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		processor.send_no_pub(event);
		process_machine(current_machine, processor, component);
	}

	return;

//	ros_decision_making_init(argc, argv);
//	startSystem(&component);

}

//void startSystem(ComponentMain* component){
//
//	RosEventQueue events;
//	CallContext context;
//	context.createParameters(new Params(component));
//	//events.async_spin();
//	component->set_events(&events);
//	LocalTasks::registration("SYS_OFF",state_OFF);
//	LocalTasks::registration("SYS_INIT",state_INIT);
//	LocalTasks::registration("SYS_READY",state_READY);
//	LocalTasks::registration("SYS_EMERGENCY",state_EMERGENCY);
//	LocalTasks::registration("ABL",tsk_ABL);
//	LocalTasks::registration("EventTranslator",tsk_EventTranslator);
//
//	ROS_INFO("Starting smme (System)...");
//	Fsmsmme(&context, &events);
//
//}
