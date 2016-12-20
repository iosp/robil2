#include <iostream>
#include <ros/ros.h>

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
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
		}
		catch (...) {
			ROS_ERROR("shiffon2ros::AsyncTask --- Unknown Exception");
		}
	}

	void pause(int millisec) {
		int msI = (millisec / 100), msR = (millisec % 100);
		for (int si = 0; si < msI and not comp_ptr->isClosed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->isClosed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
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
		ros::Rate IPON_rate(100);
		while (ros::ok()) {
			comp_ptr->ReadAndPub_ShiphonGPS();
			comp_ptr->ReadAndPub_ShiphonINS();
			comp_ptr->ReadAndPub_ShiphonGpsSpeed();
			IPON_rate.sleep();
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

void build_fsm(
    /*INPUT*/
    Processor& processor,
    cognitao::machine::Context& context,
    /*OUTPUT*/
    cognitao::io::compiler::CompilationObjectsCollector& collector,
    cognitao::io::compiler::CompiledMachine& ready_machine
)
{
    std::stringstream mission_description_stream;
    mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
		    << endl << "<tao>" << endl << "	<machines>" << endl
		    << "		<machine file=\"${rospack:shiffon2ros}/src/xml/shiffon2ros.xml\"/>"
		    << endl << "		<root>shiffon2ros</root>" << endl << "	</machines>"
		    << endl << "</tao>" << endl;

    
    cognitao::io::parser::xml::XMLParser parser;
    cognitao::io::parser::MachinesCollection machines;
    try {
	    machines = parser.parse(mission_description_stream, context.str());
    } catch (const cognitao::io::parser::ParsingError& error) {
	    std::cerr << "ParsingError:" << endl << error.message << endl;
	    return;
    }

    cognitao::io::compiler::Compiler compiler;
    
    compiler.add_builder(
		    cognitao::io::compiler::MachineBuilder::Ptr(
				    new cognitao::io::compiler::fsm::FsmBuilder(processor)));
    compiler.add_builder(
		    cognitao::io::compiler::MachineBuilder::Ptr(
				    new cognitao::io::compiler::ftt::FttBuilder(processor)));

    
    try {
	    ready_machine = compiler.compile(machines, collector);
    } catch (const cognitao::io::compiler::CompilerError& error) {
	    std::cerr << "CompilerError:" << endl << error.message << endl;
	    return;
    }
  
    cout << endl << endl;
}


void run_fsm(
    /*INPUT*/
    Processor& processor,
    cognitao::machine::Context& context,
    cognitao::bus::EventQueue& events,
    ComponentMain& component,
    cognitao::io::compiler::CompiledMachine& ready_machine
)
{
  
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
  
}



void runComponent(int argc, char** argv, ComponentMain& component) {

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);
	component.set_events(&events);
	cognitao::machine::Context context("shiffon2ros");
	
	Processor processor(events);
	cognitao::io::compiler::CompilationObjectsCollector collector;
	cognitao::io::compiler::CompiledMachine ready_machine;

// 	build_fsm( processor, context, collector, ready_machine );
// 	run_fsm( processor, context, events, component, ready_machine );
	
	TaskInit _init_state(&component, &processor, "/shiffon2ros");
	_init_state.run();
	
	TaskReady _ready_state(&component, &processor, "/shiffon2ros");
	_ready_state.run();
	
	return;

}
