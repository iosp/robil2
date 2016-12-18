#include <iostream>
#include <ros/ros.h>


using namespace std;
#include "ComponentStates.h"
#include "AblManager.h"
#include "EventTranslator.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current SMME task: " << current_task); \
					DELETE(systask_ptr) \
					systask_ptr = new Y; \
					systask_ptr->start(); \
					continue;}
#define EVENT(X) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(context))
#define RAISE(X) processor_ptr->bus_events << EVENT(X)

class SysTask {
protected:
	ComponentMain * comp_ptr;
	boost::thread run_thread;
	Processor * processor_ptr;
	std::string context;
	boost::mutex m;
public:
	SysTask(ComponentMain* comp, Processor * processor, std::string event_context) :
			comp_ptr(comp), processor_ptr(processor),  context(event_context) {
		boost::mutex::scoped_lock l(m);
	}

	SysTask* start(){
		run_thread = boost::thread(boost::bind(&SysTask::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	virtual void run() {
		pause(10000);
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][smme-glob::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][smme-glob::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("SMME::SysTask --- Unknown Exception");
//			cout<<"[e][smme-glob::AsyncTask] unknown exception"<<endl;
		}
	}

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
//		run_thread = boost::thread(boost::bind(&SysInitTask::run, this));
	}

	virtual void run() {
		//pause(10000);
		ROS_INFO("SMME INIT");
		RAISE("/EndOfCoreSystemInit");
	}

	virtual ~SysInitTask() {}
};

class SysReadyTask : public SysTask {
protected:
	AblManager * abl_m;

public:
	SysReadyTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : SysTask(comp,processor,current_context) {
		abl_m = new AblManager(comp_ptr);
//		run_thread = boost::thread(boost::bind(&SysReadyTask::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME at Ready");
		pause(10000);
		abl_m->listen(comp_ptr->events());
		EventTranslator(comp_ptr, comp_ptr->events());
	}

	virtual ~SysReadyTask() {
		delete abl_m;
	}
};

class SysEmergencyTask : public SysTask {
public:
	SysEmergencyTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : SysTask(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&SysEmergencyTask::run, this));
	}

	virtual void run() {
		pause(10000);
	}

	virtual ~SysEmergencyTask()	{}
};



SysTask* systask_ptr;

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
//				if (current_task == "off" || current_task == "init" || current_task == "ready" || current_task == "emergency")
//					systask_ptr->assign(current_event_context, current_task);
//				ROS_WARN_STREAM(" Current state: " << current_task);
//				ROS_INFO_STREAM(
//						" Current event context: " << current_event_context);
				RESET("off", SysTask(&component, &processor, current_event_context))
				RESET("init", SysInitTask(&component, &processor, current_event_context))
				RESET("ready", SysReadyTask(&component, &processor, current_event_context))
				RESET("emergency", SysEmergencyTask(&component, &processor, current_event_context))
			}
		}
	}
}

void runComponent(int argc, char** argv, ComponentMain& component){

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:smme}/src/xml/smme.xml\"/>"
			<< endl << "		<root>smme</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("smme");
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
//	systask_ptr = new SysTask(&component, &processor);
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

//	delete(systask_ptr);

	return;

//	ros_decision_making_init(argc, argv);
//	startSystem(&component);

}
