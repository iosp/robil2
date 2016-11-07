#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
#include <gazebo_msgs/GetModelState.h>
#include "aux_functions.h"
#include <bondcpp/bond.h>
#include <llc/ControlParamsConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;
//using namespace decision_making;
#include "ComponentStates.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current LLC task: " << current_task); \
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

double k_emrg = 1;
double hb_time = 0;
const double t_out = 20.0;
static double Kp = 1.3, Kd = 0.0, Ki = 0.0; /* PID constants of linear x */
static double Kpz = -1.8, Kdz = 0.01, Kiz = -0.1; /* PID constants of angular z */

void hb_callback(const std_msgs::Bool &msg) {
	if (msg.data == true)
		hb_time = ros::Time::now().toSec();
	if (msg.data == false)
		hb_time = t_out + 1.0;
}

void llc_status_callback(const std_msgs::Float64 &msg) {
	k_emrg = msg.data;
}

geometry_msgs::Twist Translate(
		geometry_msgs::PoseWithCovarianceStamped model_state,
		geometry_msgs::Twist model_speed) {

	std_msgs::Float64 x[3];
	std_msgs::Float64 z[3];
	std_msgs::Float64 v[3];
	std_msgs::Float64 w[3];
	std_msgs::Float64 a, b, c, d;

	geometry_msgs::Twist model_coordinates;
	double model_coordinates_linear_speed;
	double model_coordinates_angular_speed;

	a.data = model_state.pose.pose.orientation.w;
	b.data = model_state.pose.pose.orientation.x;
	c.data = model_state.pose.pose.orientation.y;
	d.data = model_state.pose.pose.orientation.z;

	x[0].data = (pow(a.data, 2) + pow(b.data, 2) - pow(c.data, 2)
			- pow(d.data, 2));
	x[1].data = 2 * b.data * c.data + 2 * a.data * d.data;
	x[2].data = 2 * b.data * d.data - 2 * a.data * c.data;

	z[0].data = 2 * b.data * d.data + 2 * a.data * c.data;
	z[1].data = 2 * c.data * d.data - 2 * a.data * b.data;
	z[2].data = (pow(a.data, 2) - pow(b.data, 2) - pow(c.data, 2)
			+ pow(d.data, 2));

	v[0].data = model_speed.linear.x;
	v[1].data = model_speed.linear.y;
	v[2].data = model_speed.linear.z;

	w[0].data = model_speed.angular.x;
	w[1].data = model_speed.angular.y;
	w[2].data = model_speed.angular.z;

	model_coordinates_linear_speed = (dot_prod(x, v, 3) / norm(x, 3));
	model_coordinates_angular_speed = (dot_prod(z, w, 3) / norm(z, 3));

	model_coordinates.linear.x = model_coordinates_linear_speed;
	model_coordinates.angular.z = model_coordinates_angular_speed;

	return (model_coordinates);

}

void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config,
		uint32_t level) {

	Kp = config.linearVelocity_P;
	Ki = config.linearVelocity_I;
	Kd = config.linearVelocity_D;
	Kpz = config.angularVelocity_P;
	Kiz = config.angularVelocity_I;
	Kdz = config.angularVelocity_D;
}

class AsyncTask {
protected:
	boost::thread run_thread;
	ComponentMain* comp_ptr;
	Processor* processor_ptr;
	std::string context;
	boost::mutex m;
public:
	AsyncTask(ComponentMain* comp, Processor * processor, std::string context) :
			comp_ptr(comp), processor_ptr(processor), context(context) {}

	virtual void run() {
		std::cout << context << " ::::::: PURE VIRTUAL" << std::endl;
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
//			cout<<"[d][llc::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][llc::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("LLC::AsyncTask --- Unknown Exception");
//			cout<<"[e][llc::AsyncTask] unknown exception"<<endl;
		}
	}

	void pause(int millisec) {
		int msI = (millisec / 100), msR = (millisec % 100);
		for (int si = 0; si < msI and not comp_ptr->isClosed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->isClosed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void offTask() {
		ROS_INFO("LLC OFF");
		//diagnostic_msgs::DiagnosticStatus status;
		//comp_ptr->publishDiagnostic(status);
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
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("LLC at Init");
//		while (!boost::this_thread::interruption_requested() and ros::ok()) {
//		 boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//		}

//		pause(10000);
		RAISE("/llc/EndOfInit");
	}

	virtual ~TaskInit() {}
};

class TaskReady: public AsyncTask {
public:
	TaskReady(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("LLC at Ready");

		double integral[2] = { }; // integration part //
		double der[2] = { }; // the derivative of the error //
		double integral_limit[2] = { 1, -0.3 }; // emergency stop //
		double angular_filter[1024] = { };
		double old_err = 0;
		int E_stop = 1;
		comp_ptr->t_flag = 0;

#ifdef LLC_USE_LOCALIZATION

		Kp = 0.12; Kd = 0.0; Ki = 0.1;
		Kpz = -1.32; Kdz = 0.0; Kiz = -0.3;

		geometry_msgs::Twist per_speed;
		geometry_msgs::PoseWithCovarianceStamped per_location;

#endif

		config::LLC::pub::EffortsSt Steering_rate; // steering rate  +- 1 //
		config::LLC::pub::EffortsTh Throttle_rate; // Throttle rate  +- 1 //
		sensor_msgs::JointState Blade_pos;

		geometry_msgs::TwistStamped cur_error; // stores the current error signal //
		geometry_msgs::TwistStamped old_error; // stores the last error signal //
		geometry_msgs::Twist t; // used for co-ordinates transform //
		ros::Subscriber link_to_platform;
		ros::Subscriber link_to_comm_check;
		ros::Publisher linear_error_publisher;
		ros::Publisher angular_error_publisher;
		ros::Publisher debug_publisher;
		ros::NodeHandle n;
		link_to_platform = n.subscribe("/Sahar/link_with_platform", 100,
				hb_callback);
		link_to_comm_check = n.subscribe("/llc_status", 100,
				llc_status_callback);
		linear_error_publisher = n.advertise<std_msgs::Float64>("/linear_error",
				100);
		angular_error_publisher = n.advertise<std_msgs::Float64>(
				"/angular_error", 100);
		debug_publisher = n.advertise<std_msgs::Float64>("/or_debug", 100);
		comp_ptr->WPD_desired_speed.twist.linear.x = 0;
		comp_ptr->WPD_desired_speed.twist.angular.z = 0;
		comp_ptr->WSM_desired_speed.twist.linear.x = 0;
		comp_ptr->WSM_desired_speed.twist.angular.z = 0;

		// PID loop //

		while (!hb_time)
			;

		ROS_INFO("Connected with platform");
		/*
		 * TODO: Diagnostics about connection
		 */

		//
		 //  PID LOOP
		 //

		while ((ros::Time::now().toSec() - hb_time) < t_out) {
			if (comp_ptr->isClosed() || !ros::ok()) { // checks whether the line is empty, or node failed //
				ROS_INFO("STOPPED");
			}

			// get measurements and calculate error signal //

#ifndef LLC_USE_LOCALIZATION

			ros::ServiceClient gmscl = n.serviceClient<
					gazebo_msgs::GetModelState>("/gazebo/get_model_state");
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name = "Sahar";
			gmscl.call(getmodelstate);
			geometry_msgs::PoseWithCovarianceStamped gigi;
			gigi.pose.pose = getmodelstate.response.pose;
			t = Translate(gigi, getmodelstate.response.twist);

#else

			per_location = comp_ptr->Per_pose;
			per_speed = comp_ptr->Per_measured_speed;
			t = Translate(per_location, per_speed);

#endif

			if (comp_ptr->WPD_desired_speed.twist.linear.x
					|| comp_ptr->WPD_desired_speed.twist.angular.z) {
				cur_error.twist.linear.x =
						(comp_ptr->WPD_desired_speed.twist.linear.x)
								- t.linear.x;
				cur_error.twist.angular.z =
						((comp_ptr->WPD_desired_speed.twist.angular.z)
								- t.angular.z);
			} else {
				cur_error.twist.linear.x =
						(comp_ptr->WSM_desired_speed.twist.linear.x)
								- t.linear.x;
				cur_error.twist.angular.z =
						((comp_ptr->WSM_desired_speed.twist.angular.z)
								- t.angular.z);
			}
			std_msgs::Float64 angular_error;
			std_msgs::Float64 linear_error;
			std_msgs::Float64 debug_f;
			angular_error.data = cur_error.twist.angular.z;
			linear_error.data = cur_error.twist.linear.x;

			debug_f.data = comp_ptr->WPD_desired_speed.twist.linear.x;
			linear_error_publisher.publish(linear_error);
			angular_error_publisher.publish(angular_error);
			debug_publisher.publish(debug_f);

			Push_elm(angular_filter, 1024, cur_error.twist.angular.z);
			//cur_error.twist.angular.z = _medianfilter(angular_filter,201);
			cur_error.twist.angular.z = avg_filter(angular_filter, 1024.0);

			cur_error.twist.linear.x = (1 - 0.125) * old_err
					+ cur_error.twist.linear.x * (0.125);
			old_err = cur_error.twist.linear.x;

			double dt = 0.001;

			// calculate integral and derivatives //
			integral[0] += ((cur_error.twist.linear.x) * dt);
			der[0] =
					((cur_error.twist.linear.x - old_error.twist.linear.x) / dt);
			integral[1] += ((cur_error.twist.angular.z) * dt);
			der[1] = ((cur_error.twist.angular.z - old_error.twist.angular.z)
					/ dt);
			for (int k = 0; k < 2; k++) {
				if (integral[k] > integral_limit[k])
					integral[k] = integral_limit[k];
				else if (integral[k] < -integral_limit[k])
					integral[k] = -integral_limit[k];
			}

			//Kpz=Kpz*(1+abs(t.linear.x));
			//ROS_INFO("%f",Throttle_rate.data);

			//Steering_rate.data = E_stop*(Kpz*cur_error.twist.angular.z + Kiz*integral[1] - Kdz*der[1]) ;

			Throttle_rate.data = (Kp * cur_error.twist.linear.x
					+ Ki * integral[0] + Kd * der[0]) * k_emrg;
			Steering_rate.data = (Kpz * cur_error.twist.angular.z + Kdz * der[1]
					+ Kiz * integral[1]) * k_emrg;

			//or controlers test
			 if(comp_ptr->WPD_desired_speed.twist.angular.z>0){
			 Steering_rate.data = E_stop*(Kpz*(cur_error.twist.angular.z+0.2) + Kiz*integral[1] - Kdz*der[1]) ;
			 }
			 if(comp_ptr->WPD_desired_speed.twist.angular.z<0){
			 Steering_rate.data = E_stop*(Kpz*(cur_error.twist.angular.z-0.2) + Kiz*integral[1] - Kdz*der[1]) ;
			 }
			 if(comp_ptr->WPD_desired_speed.twist.angular.z==0){

			 }


			/*
			 // stops (or's test)
			 if((comp_ptr->WPD_desired_speed.twist.linear.x==0 && comp_ptr->WPD_desired_speed.twist.angular.z==0)|| !(comp_ptr->WPD_desired_speed.twist.linear.x || comp_ptr->WPD_desired_speed.twist.angular.z )){
			 Throttle_rate.data = 0;
			 Steering_rate.data = 0;
			 }
			 */
			// publish //
			comp_ptr->publishEffortsTh(Throttle_rate);
			comp_ptr->publishEffortsSt(Steering_rate);

			// WSM blade controller //

			if (comp_ptr->t_flag) {

				Blade_pos.name.clear();
				Blade_pos.name = comp_ptr->Blade_angle.name;
				Blade_pos.position.clear();
				for (int i = 0; i < comp_ptr->Blade_angle.position.size(); i++)
					Blade_pos.position.push_back(
							comp_ptr->Blade_angle.position[i]);
				comp_ptr->publishEffortsJn(Blade_pos);
				comp_ptr->t_flag = 0;
			}

			// calibrate the error //
			old_error.twist.angular.z = cur_error.twist.angular.z;
			old_error.twist.linear.x = cur_error.twist.linear.x;

			if (!E_stop)
				break;
			pause(10); // wait dt time to recalculate error //

			//usleep(100000);
		}
		ROS_INFO("cannot connect with platform");
		/*
		 * TODO: break bond diagnostics
		 */
		// END PID loop
	}

	virtual ~TaskReady() {}
};

class TaskStandby: public AsyncTask {
public:
	TaskStandby(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("LLC at Standby");
//		while (!boost::this_thread::interruption_requested() and ros::ok()) {
//		 boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//		}

//		pause(10000);
//		cognitao::bus::Event ev_bus_event(
//				cognitao::bus::Event::name_t("/llc/Activation"),
//				cognitao::bus::Event::channel_t(""),
//				cognitao::bus::Event::context_t(context));
//		processor_ptr->bus_events << ev_bus_event;
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
//				ROS_INFO_STREAM(" Current event context: " << current_event_context);
				if (current_task == "off")
					task_ptr->offTask();
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
//
//FSM(llc_ON)
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
//				FSM_ON_EVENT("/llc/Standby", FSM_NEXT(STANDBY));
//			}
//		}
//		FSM_STATE(STANDBY)
//		{
//			FSM_CALL_TASK(STANDBY)
//			FSM_TRANSITIONS{
//				FSM_ON_EVENT("/llc/Resume", FSM_NEXT(READY));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(llc)
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
//				FSM_ON_EVENT("/llc/Activation", FSM_NEXT(ON));
//			}
//		}
//		FSM_STATE(ON)
//		{
//			FSM_CALL_FSM(llc_ON)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
//				FSM_ON_EVENT("/llc/Shutdown", FSM_NEXT(OFF));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
//	ROS_INFO("LLC OFF");
//	//diagnostic_msgs::DiagnosticStatus status;
//	//COMPONENT->publishDiagnostic(status);
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
//	//PAUSE(10000);
//	ROS_INFO("LLC Init");
//	Event e("EndOfInit");
//	events.raiseEvent(e);
//	return TaskResult::SUCCESS();
//}
//TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
//
//	#define LLC_USE_LOCALIZATION
//
//	ROS_INFO("LLC Ready");
//
//	double integral [2] = {} ; 								/* integration part */
//	double der [2] = {} ;  									/* the derivative of the error */
//	double integral_limit [2] = {1,-0.3};					/* emergency stop */
//	double angular_filter[1024] = {} ;
//	double old_err = 0;
//	int E_stop = 1;
//	COMPONENT->t_flag = 0 ;
//
//#ifdef LLC_USE_LOCALIZATION
//
//	Kp = 0.12 ; Kd = 0.0 ; Ki = 0.1 ;
//	Kpz = -1.32 ; Kdz = 0.0 ; Kiz = -0.3 ;
//
//	geometry_msgs::Twist per_speed ;
//	geometry_msgs::PoseWithCovarianceStamped per_location ;
//
//#endif
//
//	config::LLC::pub::EffortsSt Steering_rate ; 		/* steering rate  +- 1 */
//	config::LLC::pub::EffortsTh Throttle_rate ;			/* Throttle rate  +- 1 */
//	sensor_msgs::JointState Blade_pos;
//
//	geometry_msgs::TwistStamped cur_error ; 			/* stores the current error signal */
//	geometry_msgs::TwistStamped old_error ; 			/* stores the last error signal */
//	geometry_msgs::Twist t ;							/* used for co-ordinates transform */
//	ros::Subscriber link_to_platform ;
//	ros::Subscriber link_to_comm_check;
//	ros::Publisher linear_error_publisher;
//	ros::Publisher angular_error_publisher;
//	ros::Publisher debug_publisher;
//	ros::NodeHandle n;
//	link_to_platform = n.subscribe("/Sahar/link_with_platform" , 100, hb_callback);
//	link_to_comm_check = n.subscribe("/llc_status" , 100, llc_status_callback);
//	linear_error_publisher = n.advertise<std_msgs::Float64>("/linear_error", 100);
//        angular_error_publisher = n.advertise<std_msgs::Float64>("/angular_error", 100);
//	debug_publisher = n.advertise<std_msgs::Float64>("/or_debug", 100);
//	COMPONENT->WPD_desired_speed.twist.linear.x = 0;
//	COMPONENT->WPD_desired_speed.twist.angular.z = 0;
//	COMPONENT->WSM_desired_speed.twist.linear.x = 0;
//	COMPONENT->WSM_desired_speed.twist.angular.z = 0;
//
//	/* PID loop */
//
//	while (!hb_time);
//
//		ROS_INFO("Connected with platform");
//		/*
//		 * TODO: Diagnostics about connection
//		 */
//
///*
// *  PID LOOP
// */
//
//	while((ros::Time::now().toSec() - hb_time) < t_out){
//		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
//			ROS_INFO("STOPPED");
//			return TaskResult::TERMINATED();
//		}
//
//	/* get measurements and calculate error signal */
//
//#ifndef LLC_USE_LOCALIZATION
//
//	    ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
//	    gazebo_msgs::GetModelState getmodelstate;
//	    getmodelstate.request.model_name ="Sahar";
//	    gmscl.call(getmodelstate);
//	    geometry_msgs::PoseWithCovarianceStamped gigi;
//	    gigi.pose.pose = getmodelstate.response.pose;
//	    t = Translate(gigi, getmodelstate.response.twist);
//
//#else
//
//	    per_location = COMPONENT->Per_pose ;
//	    per_speed = COMPONENT->Per_measured_speed;
//	    t = Translate(per_location, per_speed);
//#endif
//
//
//			if(COMPONENT->WPD_desired_speed.twist.linear.x ||COMPONENT->WPD_desired_speed.twist.angular.z ){
//				cur_error.twist.linear.x = (COMPONENT->WPD_desired_speed.twist.linear.x) - t.linear.x;
//				cur_error.twist.angular.z = ((COMPONENT->WPD_desired_speed.twist.angular.z) - t.angular.z);
//			}
//			else{
//				cur_error.twist.linear.x = (COMPONENT->WSM_desired_speed.twist.linear.x) - t.linear.x;
//				cur_error.twist.angular.z = ((COMPONENT->WSM_desired_speed.twist.angular.z) - t.angular.z);
//			}
//	std_msgs::Float64 angular_error;
//	std_msgs::Float64 linear_error;
//	std_msgs::Float64 debug_f;
//	angular_error.data=cur_error.twist.angular.z;
//	linear_error.data=cur_error.twist.linear.x;
//
//	debug_f.data=COMPONENT->WPD_desired_speed.twist.linear.x;
//	linear_error_publisher.publish(linear_error);
//	angular_error_publisher.publish(angular_error);
//	debug_publisher.publish(debug_f);
//
//			Push_elm(angular_filter,1024,cur_error.twist.angular.z);
//			//cur_error.twist.angular.z = _medianfilter(angular_filter,201);
//			cur_error.twist.angular.z = avg_filter(angular_filter , 1024.0);
//
//			cur_error.twist.linear.x = (1-0.125)*old_err + cur_error.twist.linear.x*(0.125);
//			old_err = cur_error.twist.linear.x ;
//
//			double dt = 0.001 ;
//
//	/* calculate integral and derivatives */
//	integral[0] += ((cur_error.twist.linear.x )* dt);
//	der[0] = ((cur_error.twist.linear.x - old_error.twist.linear.x)/dt);
//	integral[1] += ((cur_error.twist.angular.z)* dt);
//	der[1] = ((cur_error.twist.angular.z - old_error.twist.angular.z)/dt);
//	for(int k = 0 ; k < 2 ; k++){
//		if(integral[k] > integral_limit[k] )
//				integral[k] = integral_limit[k] ;
//		else if (integral[k] < -integral_limit[k])
//				integral[k] = -integral_limit[k] ;
//	}
//
//
//
//		//Kpz=Kpz*(1+abs(t.linear.x));
////ROS_INFO("%f",Throttle_rate.data);
//
//		//Steering_rate.data = E_stop*(Kpz*cur_error.twist.angular.z + Kiz*integral[1] - Kdz*der[1]) ;
//
//
//	Throttle_rate.data =( Kp*cur_error.twist.linear.x + Ki*integral[0] + Kd*der[0] )*k_emrg;
//	Steering_rate.data =( Kpz*cur_error.twist.angular.z+Kdz*der[1]+Kiz*integral[1] )*k_emrg;
//
//
//
///*/or controlers test
//if(COMPONENT->WPD_desired_speed.twist.angular.z>0){
//Steering_rate.data = E_stop*(Kpz*(cur_error.twist.angular.z+0.2) + Kiz*integral[1] - Kdz*der[1]) ;
//}
//if(COMPONENT->WPD_desired_speed.twist.angular.z<0){
//Steering_rate.data = E_stop*(Kpz*(cur_error.twist.angular.z-0.2) + Kiz*integral[1] - Kdz*der[1]) ;
//}
//if(COMPONENT->WPD_desired_speed.twist.angular.z==0){
//
//}
//*/
//
//
//	/*
//// stops (or's test)
//	if((COMPONENT->WPD_desired_speed.twist.linear.x==0 && COMPONENT->WPD_desired_speed.twist.angular.z==0)|| !(COMPONENT->WPD_desired_speed.twist.linear.x ||COMPONENT->WPD_desired_speed.twist.angular.z )){
//	Throttle_rate.data = 0;
//	Steering_rate.data = 0;
//	}
//*/
//	/* publish */
//		COMPONENT->publishEffortsTh(Throttle_rate);
//		COMPONENT->publishEffortsSt(Steering_rate);
//
//
//	/* WSM blade controller */
//
//	if(COMPONENT->t_flag){
//
//		Blade_pos.name.clear();
//		Blade_pos.name = COMPONENT->Blade_angle.name;
//		Blade_pos.position.clear();
//	for(int i = 0 ; i < COMPONENT->Blade_angle.position.size(); i++)
//		Blade_pos.position.push_back(COMPONENT->Blade_angle.position[i]);
//		COMPONENT->publishEffortsJn(Blade_pos);
//		COMPONENT->t_flag = 0 ;
//	}
//
//	/* calibrate the error */
//	old_error.twist.angular.z = cur_error.twist.angular.z ;
//	old_error.twist.linear.x = cur_error.twist.linear.x ;
//
//		if(!E_stop)
//			break ;
//		PAUSE(10);		/* wait dt time to recalculate error */
//
//		//usleep(100000);
//	}
//	ROS_INFO("cannot connect with platform");
//	/*
//	 * TODO: break bond diagnostics
//	 */
//	// END PID loop
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
//	//PAUSE(10000);
//	//Event e("Activation");
//	//events.raiseEvent(e);
//	ROS_INFO("LLC Standby");
//
//	return TaskResult::SUCCESS();
//}

void runComponent(int argc, char** argv, ComponentMain& component) {

	ros::NodeHandle node;
	cognitao::bus::RosEventQueue events(node, NULL, 1000,
			"/robil/event_bus/events");

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:llc}/src/xml/llc.xml\"/>" << endl
			<< "		<root>llc</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("llc");
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::core::MachinesCollection machines;
	try {
		machines = parser.parse(mission_description_stream, context.str());
	} catch (const cognitao::io::parser::core::ParsingError& error) {
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

	boost::posix_time::time_duration max_wait_duration(0, 0, 5, 0);
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
//			//cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		processor.send_no_pub(event);
		process_machine(current_machine, processor, component);
	}

//	delete(task_ptr);

	return;

//	ros::init(argc, argv, "llc");
//	ros_decision_making_init(argc, argv);
//	RosEventQueue events;
//	CallContext context;
//	context.createParameters(new Params(&component));
//	//events.async_spin();
//	LocalTasks::registration("OFF",state_OFF);
//	LocalTasks::registration("INIT",state_INIT);
//	LocalTasks::registration("READY",state_READY);
//	LocalTasks::registration("STANDBY",state_STANDBY);
//
//    dynamic_reconfigure::Server<llc::ControlParamsConfig> server;
//    dynamic_reconfigure::Server<llc::ControlParamsConfig>::CallbackType f;
//
//    f = boost::bind(&dynamic_Reconfiguration_callback, _1, _2);
//    server.setCallback(f);
//
//	ROS_INFO("Starting llc...");
//	Fsmllc(&context, &events);

}
