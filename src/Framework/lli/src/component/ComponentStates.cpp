#include <sstream>
#include <iostream>

#include <ros/ros.h>
//#include <cognitao/bus/events_bus.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
using namespace std;
#include "ComponentStates.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current LLI task: " << current_task); \
					DELETE(task_ptr) \
					task_ptr = new Y; \
					task_ptr->start(); \
					continue;}
#define EVENT(X) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(context))
#define RAISE(X) processor_ptr->bus_events << EVENT(X)

/*
 * OLD B
 class Params: public CallContextParameters{
 public:
 ComponentMain* comp;
 Params(ComponentMain* comp):comp(comp){}
 std::string str()const{return "";}
 };

 //// ============== WRITE FSM HERE ========================= /////
 FSM(lli_ON)
 {
 FSM_STATES
 {
 INIT,
 READY,
 STANDBY
 }
 FSM_START(INIT);
 FSM_BGN
 {
 FSM_STATE(INIT)
 {
 FSM_CALL_TASK(INIT)
 FSM_TRANSITIONS
 {
 FSM_ON_EVENT("/EndOfInit", FSM_NEXT(STANDBY));
 }
 }
 FSM_STATE(READY)
 {
 FSM_CALL_TASK(READY)
 FSM_TRANSITIONS{
 FSM_ON_EVENT("/lli/Standby", FSM_NEXT(STANDBY));
 }
 }
 FSM_STATE(STANDBY)
 {
 FSM_CALL_TASK(STANDBY)
 FSM_TRANSITIONS{
 FSM_ON_EVENT("/lli/Resume", FSM_NEXT(READY));
 FSM_ON_EVENT("/lli_ready", FSM_NEXT(READY));
 }
 }

 }
 FSM_END
 }

 FSM(lli)
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
 FSM_CALL_TASK(OFF)
 FSM_TRANSITIONS
 {
 FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
 FSM_ON_EVENT("/lli/Activation", FSM_NEXT(ON));
 }
 }
 FSM_STATE(ON)
 {
 FSM_CALL_FSM(lli_ON)
 FSM_TRANSITIONS
 {
 FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
 FSM_ON_EVENT("/lli/Shutdown", FSM_NEXT(OFF));
 }
 }

 }
 FSM_END
 }

 TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
 COMPONENT->releaseDriverAndManipulator();
 return TaskResult::SUCCESS();
 }
 TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
 int counter=0;
 COMPONENT->workerFunc();
 ros::Duration oneSec(1.0);
 oneSec.sleep();
 while (COMPONENT->IsCLLIStillInInit()){
 if (counter > 10000) break;
 oneSec.sleep();
 counter++;
 }
 if (counter > 10000){
 printf("LLI STOPPED WO INITIALIZATION COMPLETED\n");
 Event e("/lli/Shutdown");
 events.raiseEvent(e);
 return TaskResult::FAIL();
 }

 Event e("EndOfInit");
 events.raiseEvent(e);
 return TaskResult::SUCCESS();
 }
 TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
 if(!ros::ok()){			// checks whether the node failed //
 ROS_INFO("LLI STOPPED");
 Event e("/lli/Shutdown");
 events.raiseEvent(e);
 return TaskResult::TERMINATED();
 }

 return TaskResult::SUCCESS();
 }

 TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
 ros::Rate r(10);
 COMPONENT->setReady();
 COMPONENT->checkReady();
 while (COMPONENT->StateNotReady()){
 COMPONENT->checkReady();
 }
 Event e("/lli_ready");
 events.raiseEvent(e);
 return TaskResult::SUCCESS();
 }
 */

class AsyncTask {
protected:
	boost::thread run_thread;
	ComponentMain* comp_ptr;
	Processor* processor_ptr;
	std::string context;
	boost::mutex m;
public:
	AsyncTask(ComponentMain* comp, Processor * processor, std::string context) :
			comp_ptr(comp), processor_ptr(processor), context(context) {
		boost::mutex::scoped_lock l(m);
	}

	virtual void run() {
		comp_ptr->releaseDriverAndManipulator();
	}

	AsyncTask* start(){
		run_thread = boost::thread(boost::bind(&AsyncTask::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][lli::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][lli::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("LLI::AsyncTask --- Unknown Exception");
//			cout<<"[e][lli::AsyncTask] unknown exception"<<endl;
		}
	}

	void offTask() {
		comp_ptr->releaseDriverAndManipulator();
	}

	virtual ~AsyncTask() {
		run_thread.interrupt();
		run_thread.join();
		comp_ptr = NULL;
		processor_ptr = NULL;
	}
};

class TaskInit: public AsyncTask {
public:
	TaskInit(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {
//		run_thread = boost::thread(boost::bind(&TaskInit::run, this));
	}

	virtual void run() {
		ROS_INFO("LLI at Init");
		int counter = 0;
		comp_ptr->workerFunc();
		ros::Duration oneSec(1.0);
		oneSec.sleep();
		while (comp_ptr->IsCLLIStillInInit()) {
			if (counter > 10000)
				break;
//			cout << "===COUNTER=== " << counter << endl;
			oneSec.sleep();
			counter++;
		}
		if (counter > 10000) {
			printf("LLI STOPPED WO INITIALIZATION COMPLETED\n");
			RAISE("/lli/Shutdown");
		}

		RAISE("/lli/EndOfInit");
	}

	virtual ~TaskInit() {}
};

class TaskReady: public AsyncTask {
public:
	TaskReady(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {
//		run_thread = boost::thread(boost::bind(&TaskReady::run, this));
	}

	virtual void run() {
		ROS_INFO("LLI at Ready");
		if (!ros::ok()) {
			ROS_INFO("LLI STOPPED");
			RAISE("/lli/Shutdown");
		}
	}

	virtual ~TaskReady() {}
};

class TaskStandby: public AsyncTask {
public:
	TaskStandby(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {
//		run_thread = boost::thread(boost::bind(&TaskStandby::run, this));
	}

	virtual void run() {
		ROS_INFO("LLI at Standby");
		ros::Rate r(10);
		comp_ptr->setReady();
		comp_ptr->checkReady();
		while (comp_ptr->StateNotReady())
			comp_ptr->checkReady();
		RAISE("/lli/Ready");
	}

	virtual ~TaskStandby() {}
};

AsyncTask* task_ptr;

void process_machine(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS: " << e_poped.str() << endl;
//		;
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
//				if (current_task == "off" || current_task == "init" || current_task == "ready" || current_task == "standby")
//					task_ptr->assign(current_event_context, current_task);
//				ROS_WARN_STREAM(" Current task: " << current_task);
//				ROS_INFO_STREAM(
//						" Current event context: " << current_event_context);
				RESET("off", AsyncTask(&component, &processor, current_event_context))
				RESET("init", TaskInit(&component, &processor, current_event_context))
				RESET("ready", TaskReady(&component, &processor, current_event_context))
				RESET("standby", TaskStandby(&component, &processor, current_event_context))
			}
		}
	}
}

void runComponent(int argc, char** argv, ComponentMain& component) {

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:lli}/src/xml/lli.xml\"/>" << endl
			<< "		<root>lli</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("lli");
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
//	task_ptr = new AsyncTask(&component, &processor);
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
//		cout << "GET: " << event << endl;
//		if (event.context().str().find(context.str()) != 0) {
//			cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		processor.send_no_pub(event);
		process_machine(current_machine, processor, component);
	}

//	delete(task_ptr);

	return;
//
//	/*
//	 * OLD B
//	 ros_decision_making_init(argc, argv);
//	 RosEventQueue events;
//	 CallContext context;
//	 context.createParameters(new Params(&component));
//	 //events.async_spin();
//	 LocalTasks::registration("OFF", state_OFF);
//	 LocalTasks::registration("INIT", state_INIT);
//	 LocalTasks::registration("READY", state_READY);
//	 LocalTasks::registration("STANDBY", state_STANDBY);
//
//	 ROS_INFO("Starting lli...");
//	 Fsmlli(&context, &events);
//	 ROS_INFO("After Starting lli...");
//	 * OLD E
//	 */
}
