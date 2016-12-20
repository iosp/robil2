/*
 * StatesMonitor.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: dan
 */

#include <cognitao/bus/events_bus.h>

namespace cognitao{
namespace monitor{
using namespace bus;

void StatesMonitor::process(){
	while(events.empty()==false){
		Event event;
		events.pop(event);
		on_event(event);
	}
}
void StatesMonitor::turn_on(const HierarchicalName& en, const ptime& stamp){
	mutex::scoped_lock l(m);
	states.insert(en);
	if(begin_time.find(en)!=begin_time.end()){
		frequency[en] = stamp - begin_time[en];
	}
	begin_time[en] = stamp;
}
void StatesMonitor::turn_off(const HierarchicalName& en, const string& res, const ptime& stamp){
	string result = (res[0]!='/'?"/":"")+res;
	mutex::scoped_lock l(m);
	if(states.find(en)==states.end()) return;
	states.erase(en);
	results[en]=result;
	end_time[en] = stamp;
	duration[en] = end_time[en] - begin_time[en];
}
void StatesMonitor::turn_off_all(const HierarchicalName& en, const string& res, const ptime& stamp){
	mutex::scoped_lock l(m);
	string result = (res[0]!='/'?"/":"")+res;
	if(states.find(en)!=states.end()){
		states.erase(en);
		results[en]=result;
		end_time[en] = stamp;
		duration[en] = end_time[en] - begin_time[en];
	}
	result = en.text+result;
	for(States::iterator i = states.begin();i!=states.end();i++){
		if( i->equals( HierarchicalName(en.text+"*") ) == false ) continue;
		const HierarchicalName& enc = *i;
		events << end_event(enc,result);
	}
}
void StatesMonitor::clean_all(const HierarchicalName& en, const ptime& stamp){
	mutex::scoped_lock l(m);
	string result = string("cleaning_by:")+en.text;
	for(States::iterator i = _find(en.text+"*",states);i!=states.end();i = _find(en.text+"*",states)){
		const HierarchicalName& enc = *i;
		results[enc]=result;
		end_time[enc] = stamp;
		duration[enc] = end_time[enc] - begin_time[enc];
		states.erase(i);
	}
}
void StatesMonitor::process_models(const Event& event){
	mutex::scoped_lock l(m);
	for(Models::iterator i=models.begin();i!=models.end();i++){
		(*i)->on_event(event,events);
	}
}
void StatesMonitor::on_event(const Event& event){

	process_models(event);

	if( event.channel() != _states_channel() )
	{
		return;
	}

	if( is_state_begin(event) ){
		turn_on(event.context(), event.stamp());
	}else
	if( is_state_endAll(event) ){
		turn_off_all(event.context(), extract_result(event), event.stamp());
	}else
	if(is_state_end(event)){
		turn_off(event.context(), extract_result(event), event.stamp());
	}else
	if(is_state_clean(event)){
		clean_all(event.context(), event.stamp());
	}
}

void StatesMonitor::add(Model* model){
	mutex::scoped_lock l(m);
	models.insert(model);
	model->init(events);
}
void StatesMonitor::add(Model& model){ add(&model); }
void StatesMonitor::remove(Model* model){
	mutex::scoped_lock l(m);
	models.erase(model);
}
void StatesMonitor::remove(Model& model){ remove(&model); }


bool StatesMonitor::is_active(const std::string& st)const{
	mutex::scoped_lock l(m);
	States::const_iterator i = _find(st, states);
	if(i!=states.end()){
		return true;
	}
	return false;
}
bool StatesMonitor::is_active(const std::string& st, std::string& full_name)const{
	mutex::scoped_lock l(m);
	States::const_iterator i = _find(st, states);
	if(i!=states.end()){
		full_name = i->text;
		return true;
	}
	return false;
}
bool StatesMonitor::is_exists(const std::string& st)const{
	mutex::scoped_lock l(m);
	Times::const_iterator i = _const_find_map(st, begin_time);
	if(i!=begin_time.end()){
		return true;
	}
	return false;
}
bool StatesMonitor::is_exists(const std::string& st, std::string& full_name)const{
	mutex::scoped_lock l(m);
	Times::const_iterator i = _const_find_map(st, begin_time);
	if(i!=begin_time.end()){
		full_name = i->first.text;
		return true;
	}
	return false;
}
posix_time::ptime StatesMonitor::get_begin_time(const std::string& st)const{
	mutex::scoped_lock l(m);
	Times::const_iterator i = _const_find_map(st, begin_time);
	if( i==begin_time.end() ) return posix_time::ptime();
	return i->second;
}
posix_time::ptime StatesMonitor::get_end_time(const std::string& st)const{
	mutex::scoped_lock l(m);
	Times::const_iterator i = _const_find_map(st, end_time);
	if( i==end_time.end() ) return posix_time::ptime();
	return i->second;
}
posix_time::time_duration StatesMonitor::get_duration(const std::string& st)const{
	mutex::scoped_lock l(m);
	Durations::const_iterator i = _const_find_map(st, duration);
	if( i==duration.end() ) return posix_time::time_duration();
	return i->second;
}
double StatesMonitor::_get_frequency(const std::string& st)const{
	Durations::const_iterator i = _const_find_map(st, frequency);
	if( i==frequency.end() ) return -1;
	return 1000000.0 / i->second.total_microseconds(); //Hz
}
double StatesMonitor::get_frequency(const std::string& st)const{
	mutex::scoped_lock l(m);
	return _get_frequency(st);
}
string StatesMonitor::get_result(const std::string& st)const{
	mutex::scoped_lock l(m);
	Results::const_iterator i = _const_find_map(st, results);
	if( i==results.end() ) return string();
	return i->second;
}
int StatesMonitor::get_statistic(const string& st, ptime& b, ptime& e, time_duration& d, double& f, string& result)const{
	mutex::scoped_lock l(m);
	return _get_statistic(st, b, e, d, f, result);
}
int StatesMonitor::_get_statistic(const string& st, ptime& b, ptime& e, time_duration& d, double& f, string& result)const{
//	mutex::scoped_lock l(m);
	int res=0;
	{
		Times::const_iterator i = _const_find_map(st, begin_time);
		if( i!=begin_time.end() ) { res=res|0x1; b = i->second; }
	}
	{
		Times::const_iterator i = _const_find_map(st, end_time);
		if( i!=end_time.end() ) { res=res|0x2; e = i->second; }
	}
	{
		Durations::const_iterator i = _const_find_map(st, duration);
		if( i!=duration.end() ) { res=res|0x4; d = i->second; }
	}
	{
		f =  _get_frequency(st);
		if( f > 0 ) { res=res|0x8; }
	}
	{
		Results::const_iterator i = _const_find_map(st, results);
		if( i!=results.end() ) { res=res|0xa; result = i->second; }
	}
	return res;
}
vector<string> StatesMonitor::_get_actives()const{
	vector<string> res;
	for(States::const_iterator i=states.begin();i!=states.end();i++){
		res.push_back(i->text);
	}
	return res;
}
vector<string> StatesMonitor::_get_actives(string name)const{
	HierarchicalName ename(name);
	vector<string> res;
	for(States::const_iterator i=states.begin();i!=states.end();i++){
		if(i->equals(ename)) res.push_back(i->text);
	}
	return res;
}
vector<string> StatesMonitor::_get_all()const{
	vector<string> res;
	for(Times::const_iterator i=begin_time.begin();i!=begin_time.end();i++){
		res.push_back(i->first.text);
	}
	return res;
}
vector<string> StatesMonitor::_get_all(string name)const{
	HierarchicalName ename(name);
	vector<string> res;
	for(Times::const_iterator i=begin_time.begin();i!=begin_time.end();i++){
		if(i->first.equals(ename)) res.push_back(i->first.text);
	}
	return res;
}
vector<string> StatesMonitor::get_actives()const{
	mutex::scoped_lock l(m);
	return _get_actives();
}
vector<string> StatesMonitor::get_actives(string name)const{
	mutex::scoped_lock l(m);
	return _get_actives(name);
}
vector<string> StatesMonitor::get_all()const{
	mutex::scoped_lock l(m);
	return _get_all();
}
vector<string> StatesMonitor::get_all(string name)const{
	mutex::scoped_lock l(m);
	return _get_all(name);
}
void StatesMonitor::end_all_states(string name, string result){
	mutex::scoped_lock l(m);
	HierarchicalName ename(name);
	vector<string> res;
	for(States::const_iterator i=states.begin();i!=states.end();i++){
		if(i->equals(ename)){
			events.rise( end_event(*i,result) );
		}
	}
}
void StatesMonitor::clear_history(string name){
	mutex::scoped_lock l(m);
	HierarchicalName ename(name);
	vector<string> b = _get_all(name);
	for(vector<string>::const_iterator i=b.begin();i!=b.end();i++){
		HierarchicalName s(*i);
		end_time.erase(s);
		duration.erase(s);
		frequency.erase(s);
		results.erase(s);
		if(states.find(s)!=states.end()) continue;
		begin_time.erase(s);
	}
}
ostream& operator<<(ostream& out, const StatesMonitor& m){
	mutex::scoped_lock l(m.m);
	using namespace boost::gregorian;
    using namespace boost::posix_time;
    out<<"States:"<<endl;
	for(StatesMonitor::States::const_iterator i=m.states.begin();i!=m.states.end();i++){
		out<<"    ";
		out<< i->str() ;
		out<<" : "<< to_simple_string( m.begin_time.at(*i) );
		out<<endl;
	}
	out<<"Results:"<<endl;
	for(StatesMonitor::Results::const_iterator i=m.results.begin();i!=m.results.end();i++){
		out<<"    ";
		out<< i->first.str();
		out<<" : "<<i->second;
		out<<" : b="<<to_simple_string( m.begin_time.at(i->first) );
		out<<" : e="<<to_simple_string( m.end_time.at(i->first) );
		out<<" : d="<<to_simple_string( m.duration.at(i->first) );
		double freq = m._get_frequency(i->first.str());
		if(freq>0)
			out<<" : f="<<freq<<" Hz" ;//<< ", " << m.frequency.at(i->first).total_microseconds()/1000000.0;
		out<<endl;
	}
	return out;
}


}
}




