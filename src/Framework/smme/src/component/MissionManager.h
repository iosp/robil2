/*
 * MissionManagment.h
 *
 *  Created on: Mar 2, 2014
 *      Author: dan
 */

#ifndef MISSIONMANAGMENT_H_
#define MISSIONMANAGMENT_H_

#include <map>
#include <deque>
#include <vector>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace boost;
using namespace boost::posix_time;


#include <tf/tf.h>
#include <robil_msgs/AssignManipulatorTask.h>
#include <robil_msgs/AssignMission.h>
#include <robil_msgs/AssignNavTask.h>
#include <robil_msgs/MissionAcceptance.h>

#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


class MissionManager{
	//================== External Types ===========================
public:
	typedef robil_msgs::AssignMission Mission;
	typedef robil_msgs::AssignNavTask NavTask;
	typedef robil_msgs::AssignManipulatorTask ManTask;
	typedef robil_msgs::MissionAcceptance MissionAcceptance;
	typedef string MissionID;
	typedef string TaskID;
	typedef string StateID;
	typedef size_t Index;
	//=============================================================

	//================== Internal Types ===========================
protected:
	struct MissionState{
		MissionID mid;
		Index tidx;
		StateID mstate;
		StateID tstate;
		MissionState(MissionID mid):mid(mid),tidx(0){}
	};
public:
	enum TASK_TYPE{
		TT_Navigation, TT_Manipulator, TT_Unknown
	};
	struct Configuration{
		StateID start_mission_state;
		StateID start_task_state;
		StateID stop_task_state;
	};
	//=============================================================

	//================== Sets =====================================
protected:
	typedef map<TaskID, NavTask> Set_NavTasks;
	typedef map<TaskID, ManTask> Set_ManTasks;
	typedef map<MissionID, Mission> Set_Missions;
	typedef map<MissionID, MissionState> Set_MissionStates;
	//=============================================================

	//================== Exceptions ===============================
public:
	struct Exception{
	};
	struct MissionIDFault:public Exception{};
	struct TaskIDFault:public Exception{};
	struct CurrentMissionIDFault:public Exception{};
	struct CurrentTaskIDFault:public Exception{};
	struct TaskIndexFault:public Exception{};
	//=============================================================


	//================== Members ==================================
public:
	Configuration conf;

protected:
	Set_NavTasks nav_tasks;
	Set_ManTasks man_tasks;
	Set_Missions missions;
	Set_MissionStates missions_states;
	MissionID current_mission;

	boost::recursive_mutex mtx;
	//=============================================================

	//==================== External Types Data Structures Utils. ==
protected:
	inline
	MissionID id(const Mission& mission){
		return mission.mission_id;
	}
	inline
	TaskID id(const NavTask& task){
		return task.task_id;
	}
	inline
	TaskID id(const ManTask& task){
		return task.task_id;
	}
	inline
	size_t tasks_count( const Mission& m)const{
		return m.tasks.size();
	}
	inline
	TaskID task_id( const Mission& m, Index i)const{
		return m.tasks[i].task_id;
	}
	//=============================================================

	//==================== Set Utils. =============================
protected:
	template <class Set>
	bool contains( const Set& set, const typename Set::key_type& key){ return set.find(key)!=set.end(); }
	template <class Set>
	void add( Set& set, const typename Set::key_type& k, const typename Set::mapped_type& v){
		typedef typename Set::value_type val;
		if( contains(set, k) ){
			set.at(k) = v;
		}else{
			set.insert( val( k, v) );
		}
	}
	//==============================================================

	//==================== Data Structures Utils. with Search ======
protected:
	size_t tasks_count(MissionID mid);
	TaskID task_id(MissionID mid, Index i);
	//==============================================================

	//==================== Assignees ===============================
protected:
	bool is_all_tasks_assigned(const Mission& mission);
public:
	void assign(const NavTask& task);
	void assign(const ManTask& task);
	MissionAcceptance assign(const Mission& mission);
	MissionAcceptance createMissionAcceptedMessage(const Mission& mission);
	MissionAcceptance createMissionRejectedMessage(const Mission& mission, int error_code);
	MissionAcceptance createMissionReassignedMessage(const Mission& mission);
	void remove(const MissionID& mid);
	//==============================================================

	//==================== Mission and Tasks Interface =============
protected:
	void start_task(const MissionID& mid);
	void stop_task(const MissionID& mid);
	void stop_mission(const MissionID& mid);
public:
	void start_mission(const MissionID& mid);
	//=================================================================

	//==================== Current mission interface ==================
public:
	bool defined(const MissionID& mid){ return mid!=""; }
	MissionState& get_current_mission();
	StateID mission_state();
	StateID task_state();
	void mission_state(StateID id);
	void task_state(StateID id);
	TaskID task_id();
	//=================================================================

	//=================== Current Mission and Task Interface ==========
public:
	StateID change_mission(const MissionID& mid);
	bool next_task();
	TASK_TYPE task_type();
	NavTask get_nav_task();
	ManTask get_man_task();
	string print_state();
	//=================================================================

};


inline
bool operator!=(const MissionManager::Mission& m1,const MissionManager::Mission& m2){ return false; //TODO
}
inline
bool operator!=(const MissionManager::NavTask& m1,const MissionManager::NavTask& m2){ return false; //TODO
}
inline
bool operator!=(const MissionManager::ManTask& m1,const MissionManager::ManTask& m2){ return false; //TODO
}



#endif /* MISSIONMANAGMENT_H_ */
