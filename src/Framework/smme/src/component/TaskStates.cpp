#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "ComponentStates.h"
#include "MissionManager.h"
#include "Types.h"

#define M_PREF string("smme/mission")
#define T_PREF string("smme/task")
#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current task: " << current_task); \
					DELETE(task_ptr) \
					task_ptr = new Y; \
					task_ptr->start(); \
					continue;}
#define EVENT(X,C) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(C))
#define RAISE(X,C) processor_ptr->bus_events << EVENT(X,C)
//#define RAISE_NO_CONTEXT(X) processor_ptr->bus_events << cognitao::bus::Event( \
//																cognitao::bus::Event::name_t(X))
//#define MISSION_ID MID_PREF(FSM_CONTEXT.parameters<TaskParams>().mission_id)

class AsyncTask {
protected:
	ComponentMain * comp_ptr;
	boost::thread run_thread, activation_thread;
	Processor * processor_ptr;
	std::string context;
	boost::mutex m;
	static bool is_active;

public:
#define MM comp_ptr->mission_manager()
#define MID string mid = MM->get_current_mission().mid;
	AsyncTask(ComponentMain* comp, Processor * processor, std::string event_context) :
		comp_ptr(comp), processor_ptr(processor),  context(event_context) {
		boost::mutex::scoped_lock l(m);
	}

	AsyncTask* start(){
		run_thread = boost::thread(boost::bind(&AsyncTask::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	virtual void run() {
		ROS_INFO("SMME task at Pending.");
		MID
		MissionManager::MissionID cmid = MM->get_current_mission().mid;
		MM->change_mission(mid);
		MM->task_state("pending");
		MM->change_mission(cmid);
		is_active = false;
		activation_thread.interrupt();
		activation_thread.join();
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][smme-task::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][smme-task::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("SMME::AsyncTask --- Unknown Exception");
//			cout<<"[e][smme-task::AsyncTask] unknown exception"<<endl;
		}
	}

	void pause(int millisecs) {
		int msI = (millisecs / 100), msR = (millisecs % 100);
		for (int si = 0; si < msI and not comp_ptr->events()->is_closed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->events()->is_closed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void active() {
		is_active = true;
		while(comp_ptr->events()->is_closed()==false and ros::ok()){
			this_thread::sleep(milliseconds(100));
		}
		ROS_INFO("active out of loop");
		if(MM->task_type()==MissionManager::TT_Navigation) RAISE("/pp/Standby", context);
		else RAISE("/wsm/Standby", context);
	}

	virtual ~AsyncTask() {
		run_thread.interrupt();
		run_thread.join();
		comp_ptr = NULL;
		processor_ptr = NULL;
	}
};

class AsyncTaskSpooling : public AsyncTask {
public:
	AsyncTaskSpooling(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncTaskSpooling::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME task at Spooling.");
		if(!is_active) activation_thread = boost::thread(boost::bind(&AsyncTask::active, this));
		MID
		MM->task_state("spooling");
		if(MM->task_type()==MissionManager::TT_Navigation){
			MissionManager::NavTask task = MM->get_nav_task();
			config::SMME::pub::GlobalPath path = extract_path(task);
			RAISE("/pp/Resume", context);
//			RAISE("/pp/StartTask");
//			RAISE("Resume", "pp");
			this_thread::sleep(milliseconds(500));
			comp_ptr->publishGlobalPath(path);
		}else
		if(MM->task_type()==MissionManager::TT_Manipulator){
			MissionManager::ManTask task = MM->get_man_task();
			RAISE("/wsm/Resume", context);
//			RAISE("Resume", "wsm");
			this_thread::sleep(milliseconds(500));
			comp_ptr->publishWorkSeqData(task);
		}else
		if(MM->task_type()==MissionManager::TT_Unknown){
			ROS_ERROR("smme: Active task type is unknown");
			RAISE(T_PREF + "/" + mid + "/AbortTask", context);
//			RAISE(mid + "/AbortTask", T_PREF);
		}else{
			ROS_ERROR("smme: Error in task type detector");
			RAISE(T_PREF + "/" + mid + "/AbortTask", context);
//			RAISE(mid + "/AbortTask", T_PREF);
		}
	}

	virtual ~AsyncTaskSpooling() {}
};

class AsyncTaskPaused : public AsyncTask {
public:
	AsyncTaskPaused(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncTaskPaused::run, this));
	}
	virtual void run() {
		ROS_INFO("SMME task at Paused.");
		if(!is_active) activation_thread = boost::thread(boost::bind(&AsyncTask::active, this));
		MM->task_state("paused");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby", context);
		}
		else {
			RAISE("/wsm/Standby", context);
		}
	}

	virtual ~AsyncTaskPaused() {}
};

class AsyncTaskAborted : public AsyncTask {
public:
	AsyncTaskAborted(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncTaskAborted::run, this));
	}

	virtual void run () {
		ROS_INFO("SMME task at Aborted.");
		if(!is_active) activation_thread = boost::thread(boost::bind(&AsyncTask::active, this));
		MID
		RAISE(mid + "/AbortMission", M_PREF);

		MM->task_state("aborted");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby", context);
		}
		else {
			RAISE("/wsm/Standby", context);
		}
	}

	virtual ~AsyncTaskAborted() {}
};

class AsyncTaskFinished : public AsyncTask {
public:
	AsyncTaskFinished(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncTaskFinished::run, this));
	}

	virtual void run () {
		ROS_INFO("SMME task at Finished.");
		if(!is_active) activation_thread = boost::thread(boost::bind(&AsyncTask::active, this));
		MM->task_state("finished");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby", context);
		}
		else {
			RAISE("/wsm/Standby", context);
		}
	//	if( MM->next_task() ){
	//		RAISE("restart");
	//	}else{
	//		RAISE("complete");
	//	}
	}

	virtual ~AsyncTaskFinished() {}
};


#include <robil_msgs/MissionState.h>
MissionManager* __mission_manager=0;
bool service_get_mission_state(robil_msgs::MissionState::Request& req,robil_msgs::MissionState::Response& res){
	if(__mission_manager){
		std::string state = __mission_manager->print_state();
		res.states = state;
		return true;
	}
	return false;
}

ros::ServiceServer ss_get_mission_state;
void startStateService(ComponentMain* component, std::string mid){
	__mission_manager = component->mission_manager();
	__mission_manager->change_mission(mid);
	ros::NodeHandle node;
	ss_get_mission_state = node.advertiseService("/mission_state",&service_get_mission_state);
}


AsyncTask * task_ptr;
bool AsyncTask::is_active = false;

void process_task(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS_TASK: " << e_poped.str() << endl;
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
//				if (current_task == "spooling" || current_task == "paused"
//						|| current_task == "aborted"
//						|| current_task == "finished"
//						|| current_task == "pending")
//					task_ptr->assign(current_event_context, current_task);
//				ROS_WARN_STREAM(" Current task: " << current_task);
//				ROS_INFO_STREAM(
//						" Current event context: " << current_event_context);
				RESET("pending", AsyncTask(&component, &processor, current_event_context))
				RESET("unloaded", AsyncTask(&component, &processor, current_event_context))
				RESET("spooling", AsyncTaskSpooling(&component, &processor,	current_event_context))
				RESET("paused", AsyncTaskPaused(&component, &processor,	current_event_context))
				RESET("aborted", AsyncTaskAborted(&component, &processor,	current_event_context))
				RESET("finished", AsyncTaskFinished(&component, &processor,	current_event_context))
			}
		}
	}
}

void TaskMachine::startTask(ComponentMain* component, std::string mission_id){

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component->set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:smme}/src/xml/smme_task.xml\" TID=\"" << mission_id << "\" />"
			<< endl << "		<root>task</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("smme/mission");
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
//	task_ptr = new AsyncTask(component, &processor);
	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine =
			ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_task(current_machine, processor, *component);

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
		process_task(current_machine, processor, *component);
	}

//	delete(task_ptr);

	return;

//	RosEventQueue events;
//	events_ptr = &events;
//	CallContext context;
//	context.push("mission");
//	context.push(mission_id);
//	context.createParameters(new TaskParams(component, mission_id));
//
//	ROS_INFO_STREAM("Starting smme (Task:"<<mission_id<<")...");
//	FsmTask(&context, &events);
//	ROS_INFO_STREAM("Stop smme (Task:"<<mission_id<<")");

}
void TaskMachine::stop(){
	static_cast<cognitao::bus::RosEventQueue*>(events_ptr)->close();
	thread.join_all();
}
