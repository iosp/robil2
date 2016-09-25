#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
using namespace std;
//using namespace decision_making;
#include "ComponentStates.h"
#include "MissionManager.h"
#include "Types.h"



//class TaskParams: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	string mission_id;
//	TaskParams():comp(0), mission_id(""){struct TaskParamsConstructorEmpty{}; throw TaskParamsConstructorEmpty();}
//	TaskParams(ComponentMain* comp, string mission_id)
//		:comp(comp), mission_id(mission_id)
//	{
//
//	}
//	std::string str()const{return "";}
//};

#define MID_PREF(X) string("/mission/")+X
#define DELETE(X) if(X){delete X; X=NULL;}
#define EVENT(X) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(context))
#define RAISE(X) processor_ptr->bus_events << EVENT(X)
#define MISSION_ID MID_PREF(FSM_CONTEXT.parameters<TaskParams>().mission_id)

class AsyncTask {
protected:
	ComponentMain * comp_ptr;
	boost::thread run_thread;
	Processor * processor_ptr;
	std::string context;

public:
#define MM comp_ptr->mission_manager()
#define MID string mid = MM->get_current_mission().mid;
	AsyncTask(ComponentMain* comp, Processor * processor,
			std::string current_context) : comp_ptr(comp),processor_ptr(processor),context(context) {
		run_thread = boost::thread(boost::bind(&AsyncTask::run, this));
	}

	virtual void run() = 0;

	void pause(int millisecs) {
		int msI = (millisecs / 100), msR = (millisecs % 100);
		for (int si = 0; si < msI and not comp_ptr->events()->is_closed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->events()->is_closed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void onPending() {
		MID
		MissionManager::MissionID cmid = MM->get_current_mission().mid;
		MM->change_mission(mid);
		MM->task_state("pending");
		MM->change_mission(cmid);
	}

	void onActive() {
		while(comp_ptr->events()->is_closed()==false and ros::ok()){
			this_thread::sleep(milliseconds(100));
		}
		if(MM->task_type()==MissionManager::TT_Navigation) RAISE("/pp/Standby");
		else RAISE("/wsm/Standby");
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
		run_thread = boost::thread(boost::bind(&AsyncTaskSpooling::run, this));
	}

	void run() {
		MID
		MM->task_state("spooling");
		if(MM->task_type()==MissionManager::TT_Navigation){
			MissionManager::NavTask task = MM->get_nav_task();
			config::SMME::pub::GlobalPath path = extract_path(task);
			RAISE("/pp/StartTask");
			this_thread::sleep(milliseconds(500));
			comp_ptr->publishGlobalPath(path);
		}else
		if(MM->task_type()==MissionManager::TT_Manipulator){
			MissionManager::ManTask task = MM->get_man_task();
			RAISE("/wsm/Resume");
			this_thread::sleep(milliseconds(500));
			comp_ptr->publishWorkSeqData(task);
		}else
		if(MM->task_type()==MissionManager::TT_Unknown){
			ROS_ERROR("smme: Active task type is unknown");
			RAISE("/smme/AbortTask");
		}else{
			ROS_ERROR("smme: Error in task type detector");
			RAISE("/smme/AbortTask");
		}
	}
};

class AsyncTaskPaused : public AsyncTask {
public:
	AsyncTaskPaused(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
		run_thread = boost::thread(boost::bind(&AsyncTaskPaused::run, this));
	}

	void run() {
		MM->task_state("paused");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby");
		}
		else {
			RAISE("/wsm/Standby");
		}
	}
};

class AsyncTaskAborted : public AsyncTask {
public:
	AsyncTaskAborted(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
		run_thread = boost::thread(boost::bind(&AsyncTaskAborted::run, this));
	}

	void run () {
		MID
		RAISE(mid + "/AbortMission");

		MM->task_state("aborted");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby");
		}
		else {
			RAISE("/wsm/Standby");
		}
	}
};

class AsyncTaskFinished : public AsyncTask {
public:
	AsyncTaskFinished(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncTask(comp,processor,current_context) {
		run_thread = boost::thread(boost::bind(&AsyncTaskFinished::run, this));
	}

	void run () {
		MM->task_state("finished");
		if(MM->task_type()==MissionManager::TT_Navigation) {
			RAISE("/pp/Standby");
		}
		else {
			RAISE("/wsm/Standby");
		}
	//	if( MM->next_task() ){
	//		RAISE("restart");
	//	}else{
	//		RAISE("complete");
	//	}
	}
};

//FSM(TaskActive)
//{
//	FSM_STATES
//	{
//		TaskSpooling,
//		TaskPaused,
//		TaskAborted,
////		TaskTryNext,
//		TaskFinished,
//	}
//	FSM_START(TaskSpooling);
//	FSM_BGN
//	{
//		FSM_STATE(TaskSpooling)
//		{
//			FSM_CALL_TASK(TaskSpooling)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/CompleteTask", FSM_NEXT(TaskFinished));
//				FSM_ON_EVENT(MISSION_ID+"/AbortTask", FSM_NEXT(TaskAborted));
//				FSM_ON_EVENT(MISSION_ID+"/PauseTask", FSM_NEXT(TaskPaused));
//			}
//		}
//		FSM_STATE(TaskPaused)
//		{
//			FSM_CALL_TASK(TaskPaused)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/CompleteTask", FSM_NEXT(TaskFinished));
//				FSM_ON_EVENT(MISSION_ID+"/AbortTask", FSM_NEXT(TaskAborted));
//				FSM_ON_EVENT(MISSION_ID+"/ResumeTask", FSM_NEXT(TaskSpooling));
//			}
//		}
//		FSM_STATE(TaskAborted)
//		{
//			FSM_RAISE(MISSION_ID+"/AbortMission")
//			FSM_CALL_TASK(TaskAborted)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskSpooling));
//			}
//		}
////		FSM_STATE(TaskTryNext)
////		{
////			FSM_CALL_TASK(TaskFinished)
////			FSM_TRANSITIONS
////			{
////				FSM_ON_EVENT("TaskFinished/restart", FSM_NEXT(TaskSpooling));
////				FSM_ON_EVENT("TaskFinished/complete", FSM_NEXT(TaskFinished));
////			}
////		}
//		FSM_STATE(TaskFinished)
//		{
//			FSM_CALL_TASK(TaskFinished)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskSpooling));
//			}
//		}
//	}
//	FSM_END
//}
//
//FSM(Task)
//{
//	FSM_STATES
//	{
//		TaskPending,
//		TaskActive,
//	}
//	FSM_START(TaskPending);
//	call_ctx.pop();
//	FSM_BGN
//	{
//		FSM_STATE(TaskPending)
//		{
//			FSM_CALL_TASK(TaskPending)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/StartTask", FSM_NEXT(TaskActive));
//			}
//		}
//		FSM_STATE(TaskActive)
//		{
//			FSM_CALL_TASK(TaskActive)
//			FSM_CALL_FSM(TaskActive)
//			FSM_TRANSITIONS
//			{
//				//FSM_ON_EVENT(MISSION_ID+"/StopTask", FSM_RAISE(MISSION_ID+"/CompleteTask"));
//				FSM_ON_EVENT(MISSION_ID+"/StopTask", FSM_NEXT(TaskPending));
//				FSM_ON_EVENT(MISSION_ID+"/RestartTask", FSM_NEXT(TaskActive));
//			}
//		}
//	}
//	FSM_END
//}
//
//#define PARAMS \
//		std::string mid = context.parameters<TaskParams>().mission_id;\
//		ComponentMain* comp = context.parameters<TaskParams>().comp;
//#define MM comp->mission_manager()
//
//TaskResult state_TaskPending(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MissionManager::MissionID cmid = MM->get_current_mission().mid;
//	MM->change_mission(mid);
//	MM->task_state("pending");
//	MM->change_mission(cmid);
//	return TaskResult::SUCCESS();
//}
//TaskResult state_TaskActive(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	while(events.isTerminated()==false and ros::ok()){
//		this_thread::sleep(milliseconds(100));
//	}
//	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
//	else events.raiseEvent("/wsm/Standby");
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_TaskSpooling(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->task_state("spooling");
//	if(MM->task_type()==MissionManager::TT_Navigation){
//		MissionManager::NavTask task = MM->get_nav_task();
//		config::SMME::pub::GlobalPath path = extract_path(task);
//		events.raiseEvent("/pp/Resume");
//		this_thread::sleep(milliseconds(500));
//		comp->publishGlobalPath(path);
//		return TaskResult::SUCCESS();
//	}else
//	if(MM->task_type()==MissionManager::TT_Manipulator){
//		MissionManager::ManTask task = MM->get_man_task();
//		events.raiseEvent("/wsm/Resume");
//		this_thread::sleep(milliseconds(500));
//		comp->publishWorkSeqData(task);
//		return TaskResult::SUCCESS();
//	}else
//	if(MM->task_type()==MissionManager::TT_Unknown){
//		ROS_ERROR("smme: Active task type is unknown");
//		events.raiseEvent("/AbortTask");
//		return TaskResult::FAIL();
//	}else{
//		ROS_ERROR("smme: Error in task type detector");
//		events.raiseEvent("/AbortTask");
//		return TaskResult::FAIL();
//	}
//	return TaskResult::SUCCESS();
//}
//TaskResult state_TaskPaused(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->task_state("paused");
//	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
//	else events.raiseEvent("/wsm/Standby");
//	return TaskResult::SUCCESS();
//}
//TaskResult state_TaskAborted(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->task_state("aborted");
//	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
//	else events.raiseEvent("/wsm/Standby");
//	return TaskResult::SUCCESS();
//}
//TaskResult state_TaskFinished(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->task_state("finished");
//	if(MM->task_type()==MissionManager::TT_Navigation)events.raiseEvent("/pp/Standby");
//	else events.raiseEvent("/wsm/Standby");
////	if( MM->next_task() ){
////		events.raiseEvent(Event("restart",context));
////	}else{
////		events.raiseEvent(Event("complete",context));
////	}
//	return TaskResult::SUCCESS();
//}


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


//void initTask(){
//	LocalTasks::registration("TaskPending",state_TaskPending);
//	LocalTasks::registration("TaskSpooling",state_TaskSpooling);
//	LocalTasks::registration("TaskPaused",state_TaskPaused);
//	LocalTasks::registration("TaskAborted",state_TaskAborted);
//	LocalTasks::registration("TaskFinished",state_TaskFinished);
//	LocalTasks::registration("TaskActive",state_TaskActive);
//}

AsyncTask * task_ptr;

void process_task(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	bool switched = false;
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
				ROS_WARN_STREAM(" Current task: " << current_task);
				ROS_INFO_STREAM(
						" Current event context: " << current_event_context);
				if (!switched) {
					task_ptr->onActive();
					switched = true;
				}
				if (current_task == "pending") {
					task_ptr->onPending();
					switched = false;
				}
				DELETE(task_ptr);
				if (current_task == "spooling")
					task_ptr = new AsyncTaskSpooling(&component, &processor,
							current_event_context);
				if (current_task == "paused")
					task_ptr = new AsyncTaskPaused(&component, &processor,
							current_event_context);
				if (current_task == "aborted")
					task_ptr = new AsyncTaskAborted(&component, &processor,
							current_event_context);
				if (current_task == "finished")
					task_ptr = new AsyncTaskFinished(&component, &processor,
							current_event_context);
			}
		}
	}
}

void TaskMachine::startTask(ComponentMain* component, std::string mission_id){

	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000,
			"/robil/event_bus/events");

	component->set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:smme}/src/xml/smme_task.xml\" MID=\"" << mission_id << "\" />"
			<< endl << "		<root>task</root>" << endl << "	</machines>" << endl
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
		cout << "GET: " << event << endl;
//		if (event.context().str().find(context.str()) != 0) {
//			cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		processor.send_no_pub(event);
		process_task(current_machine, processor, *component);
	}

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
