/*
 * MissionManager.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: dan
 */

#include "MissionManager.h"

#define SYNCHRONIZED boost::recursive_mutex::scoped_lock locker(mtx);

size_t MissionManager::tasks_count(MissionID mid) {
SYNCHRONIZED
	if (!contains(missions, mid))
		throw MissionIDFault();

	return tasks_count(missions.at(mid));
}

MissionManager::MissionAcceptance MissionManager::assign(const Mission& mission) {
SYNCHRONIZED
	MissionID mid = id(mission);
	missions[mid] = mission;
	return createMissionAcceptedMessage(mission);
}

void MissionManager::assign(const ManTask& task) {
SYNCHRONIZED
	TaskID tid = id(task);
	man_tasks[tid] = task;
}

void MissionManager::assign(const NavTask& task) {
SYNCHRONIZED
	TaskID tid = id(task);
	nav_tasks[tid] = task;
}

MissionManager::MissionAcceptance MissionManager::createMissionAcceptedMessage(const Mission& mission){
	return MissionAcceptance();
}

void MissionManager::start_task(const MissionID& mid) {
SYNCHRONIZED
	if (!contains(missions_states, mid))
		throw MissionIDFault();

	MissionState& ms = missions_states.at(mid);
	ms.tstate = conf.start_task_state;
}

void MissionManager::stop_task(const MissionID& mid) {
SYNCHRONIZED
	if (!contains(missions_states, mid))
		throw MissionIDFault();

	MissionState& ms = missions_states.at(mid);
	ms.tstate = conf.stop_task_state;
}

void MissionManager::start_mission(const MissionID& mid) {
SYNCHRONIZED
	if (contains(missions_states, mid)) {
		stop_task(mid);
		missions_states.at(mid).tidx = 0;
		start_task(mid);
	}else{
		add(missions_states, mid, MissionState(mid));
		missions_states.at(mid).mstate = conf.start_mission_state;
		start_task(mid);
	}
}

void MissionManager::stop_mission(const MissionID& mid) {
SYNCHRONIZED
	if (!contains(missions_states, mid))
		throw MissionIDFault();

	stop_task(mid);
	missions_states.erase(mid);
}

void MissionManager::remove(const MissionID& mid){
	stop_mission(mid);
}

MissionManager::MissionState& MissionManager::get_current_mission() {
SYNCHRONIZED
	if (!contains(missions_states, current_mission))
		throw CurrentMissionIDFault();

	return missions_states.at(current_mission);
}

MissionManager::StateID MissionManager::mission_state() {
SYNCHRONIZED
	return get_current_mission().mstate;
}

void MissionManager::mission_state(StateID id) {
SYNCHRONIZED
	get_current_mission().mstate = id;
}

void MissionManager::task_state(StateID id) {
SYNCHRONIZED
	get_current_mission().tstate = id;
}

MissionManager::StateID MissionManager::task_state() {
SYNCHRONIZED
	return get_current_mission().tstate;
}

MissionManager::TaskID MissionManager::task_id() {
SYNCHRONIZED
	MissionState& m = get_current_mission();
	return task_id(m.mid, m.tidx);
}

MissionManager::TaskID MissionManager::task_id(MissionID mid, Index i) {
SYNCHRONIZED
	if (i >= tasks_count(mid))
		throw TaskIndexFault();

	return task_id(missions.at(mid), i);
}

MissionManager::StateID MissionManager::change_mission(const MissionID& mid) {
SYNCHRONIZED
	if (
				defined(current_mission)
			and
				!contains(missions_states, current_mission)
	)
		throw CurrentMissionIDFault();

	if (mid == current_mission)
		return mission_state();

	if (!contains(missions_states, mid)) {
		start_mission(mid);
	}

	current_mission = mid;
	return mission_state();
}

bool MissionManager::next_task() {
SYNCHRONIZED
	MissionState& ms = get_current_mission();
	stop_task(ms.mid);
	if (ms.tidx >= tasks_count(ms.mid) - 1)
		return false;

	ms.tidx += 1;
	start_task(ms.mid);
	return true;
}

MissionManager::TASK_TYPE MissionManager::task_type() {
SYNCHRONIZED
	TaskID tid = task_id();
	if (contains(nav_tasks, tid))
		return TT_Navigation;

	if (contains(man_tasks, tid))
		return TT_Manipulator;

	return TT_Unknown;
}

MissionManager::NavTask MissionManager::get_nav_task() {
SYNCHRONIZED
	if (TT_Navigation != task_type())
		return NavTask();

	TaskID tid = task_id();
	return nav_tasks.at(tid);
}

MissionManager::ManTask MissionManager::get_man_task() {
SYNCHRONIZED
	if (TT_Manipulator != task_type())
		return ManTask();

	TaskID tid = task_id();
	return man_tasks.at(tid);
}


