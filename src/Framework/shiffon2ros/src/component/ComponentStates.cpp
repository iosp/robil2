#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
using namespace std;

#include "ComponentStates.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current shiffon2ros task: " << current_task); \
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

class AsyncTask {
protected:
	boost::thread run_thread;
	ComponentMain* comp_ptr;
	Processor* processor_ptr;
	std::string context;
	boost::mutex m;
public:
	AsyncTask(ComponentMain* comp, Processor * processor, std::string event_context) :
			comp_ptr(comp), processor_ptr(processor),  context(event_context) {
		boost::mutex::scoped_lock l(m);
	}

	AsyncTask* start(){
		run_thread = boost::thread(boost::bind(&AsyncTask::start_run, this));
//		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	virtual void run() {
		pause(10000);
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][shiffon2ros::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][shiffon2ros::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("shiffon2ros::AsyncTask --- Unknown Exception");
//			cout<<"[e][shiffon2ros::AsyncTask] unknown exception"<<endl;
		}
	}

//	void assign(std::string current_context, std::string task) {
//		run_thread.interrupt();
//		run_thread.join();
//
//		context = current_context;
//
//		if (task == "off")
//			run_thread = boost::thread(boost::bind(&AsyncTask::off, this));
//		if (task == "init")
//			run_thread = boost::thread(boost::bind(&AsyncTask::init, this));
//		if (task == "ready")
//			run_thread = boost::thread(boost::bind(&AsyncTask::ready, this));
//		if (task == "standby")
//			run_thread = boost::thread(boost::bind(&AsyncTask::standby, this));
//	}

	void pause(int millisec) {
		int msI = (millisec / 100), msR = (millisec % 100);
		for (int si = 0; si < msI and not comp_ptr->isClosed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->isClosed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void offTask() {
		pause(10000);
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
//		while (!boost::this_thread::interruption_requested() and ros::ok()) {
//			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//		}
		ROS_INFO("shiffon2ros at Init");

		comp_ptr->InitShiphonConection();
		pause(300);

		RAISE("/shiffon2ros/EndOfInit");
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
		ROS_INFO("shiffon2ros at Ready");
		while (ros::ok()) {
			comp_ptr->ReadAndPub_ShiphonGPS();
			comp_ptr->ReadAndPub_ShiphonINS();
			comp_ptr->ReadAndPub_ShiphonGpsSpeed();
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
		pause(10000);
		ROS_INFO("shiffon2ros at Standby");
	}

	virtual ~TaskStandby() {}
};

AsyncTask* task_ptr;

void process_machine(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS: " << e_poped.str() << endl;
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

//class Params: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	Params(ComponentMain* comp):comp(comp){}
//	std::string str()const{return "";}
//};

//// ============== WRITE FSM HERE ========================= /////
//FSM(shiffon2ros_ON)
//{
//	FSM_STATES
//	{
//		INIT,
//		READY,
//		STANDBY
//	}
//	FSM_START(INIT);
//	FSM_BGN
//	{
//		FSM_STATE(INIT)
//		{
//			FSM_CALL_TASK(INIT)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/EndOfInit", FSM_NEXT(READY));
//			}
//		}
//		FSM_STATE(READY)
//		{
//			FSM_CALL_TASK(READY)
//			FSM_TRANSITIONS{
//				FSM_ON_EVENT("/shiffon2ros/Standby", FSM_NEXT(STANDBY));
//			}
//		}
//		FSM_STATE(STANDBY)
//		{
//			FSM_CALL_TASK(STANDBY)
//			FSM_TRANSITIONS{
//				FSM_ON_EVENT("/shiffon2ros/Resume", FSM_NEXT(READY));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(shiffon2ros)
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
//			FSM_CALL_TASK(OFF)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
//				FSM_ON_EVENT("/shiffon2ros/Activation", FSM_NEXT(ON));
//			}
//		}
//		FSM_STATE(ON)
//		{
//			FSM_CALL_FSM(shiffon2ros_ON)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
//				FSM_ON_EVENT("/shiffon2ros/Shutdown", FSM_NEXT(OFF));
//			}
//		}
//	}
//	FSM_END
//}
//
//TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
//	PAUSE(10000);
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_INIT(string id, const CallContext& context,
//		EventQueue& events) {
//	ROS_INFO("shiffon2ros Init !!");
//
//	COMPONENT->InitShiphonConection();
//	PAUSE(300);
//
//	Event e("EndOfInit");
//	events.raiseEvent(e);
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_READY(string id, const CallContext& context,
//		EventQueue& events) {
//	ROS_INFO("shiffon2ros Ready !!");
//
//	while (ros::ok()) {
//		COMPONENT->ReadAndPub_ShiphonGPS();
//		COMPONENT->ReadAndPub_ShiphonINS();
//		COMPONENT->ReadAndPub_ShiphonGpsSpeed();
//	}
//
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_STANDBY(string id, const CallContext& context,
//		EventQueue& events) {
//	PAUSE(10000);
//	return TaskResult::SUCCESS();
//}

void runComponent(int argc, char** argv, ComponentMain& component) {

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);
	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:shiffon2ros}/src/xml/shiffon2ros.xml\"/>"
			<< endl << "		<root>shiffon2ros</root>" << endl << "	</machines>"
			<< endl << "</tao>" << endl;

<<<<<<< HEAD
TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
	ROS_INFO("shiffon2ros Ready !!");

	ros::Rate IPON_rate(100);
	while (ros::ok()) {
		COMPONENT->ReadAndPub_ShiphonGPS();
		COMPONENT->ReadAndPub_ShiphonINS();
		COMPONENT->ReadAndPub_ShiphonGpsSpeed();
		IPON_rate.sleep();
	}
	return TaskResult::SUCCESS();
}
=======
	cognitao::machine::Context context("shiffon2ros");
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
>>>>>>> origin/moving_to_new_cognitao

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

//	ros_decision_making_init(argc, argv);
//	RosEventQueue events;
//	CallContext context;
//	context.createParameters(new Params(&component));
//	//events.async_spin();
//	LocalTasks::registration("OFF", state_OFF);
//	LocalTasks::registration("INIT", state_INIT);
//	LocalTasks::registration("READY", state_READY);
//	LocalTasks::registration("STANDBY", state_STANDBY);
//
//	ROS_INFO("Starting shiffon2ros...");
//	Fsmshiffon2ros(&context, &events);
//
//	Shiphon_Ctrl * _shiphonCtrl;

}
