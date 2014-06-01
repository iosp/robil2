/*
 * PLPCompiler.cpp
 *
 *  Created on: May 7, 2014
 *      Author: dan
 */

#include "PLPCompiler.h"
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

PLPCompiler::PLPCompiler() {

}

PLPCompiler::~PLPCompiler() {
}

template<class T>
vector<T>& operator<<(vector<T>& v, const vector<T>& t){
	for(size_t i=0;i<t.size();i++) v.push_back(t[i]);
	return v;
}
template<class T>
vector<T>& operator<<(vector<T>& v, const T& t){
	v.push_back(t);
	return v;
}
template<>
vector<MonitorningScript>& operator<<(vector<MonitorningScript>& v, const MonitorningScript& t){
	if(t.lines.empty()==false) v.push_back(t);
	return v;
}
template<class T>
std::string operator<<(const std::string& in, const T& t){
	std::stringstream s; s<<in<<t; return s.str();
}

typedef vector<MonitorningScript> ScriptCollection;
typedef MonitorningScript Script;

#define TRANSForM_COLLECTION(ELEMENT)\
	Script transform(const PLP& plp, const vector<ELEMENT>& ELEMENT_##_collection, int& error_code){\
		Script res;\
		res.properties<<(string()<<"type predicate");\
		res.properties<<(string()<<"name "<<plp.name<<"_"<<#ELEMENT);\
		res.properties<<(string()<<"module "<<plp.name);\
		for(size_t i=0;i<ELEMENT_##_collection.size() and not error_code;i++)\
			res << transform(plp, ELEMENT_##_collection[i], error_code);\
		return res;\
	}

inline
bool isdef(const std::string& str){ return str!=""; }
bool isnum(const std::string& str){ return str!="" and str!="N/A"; }

#define CALL(X) if(coll==#X){return FUNC(plp.X,prop,value,TVAL);}
#define SPLIT_COLL size_t i = coll.find(",");while(i!=string::npos){ if(FUNC(plp,coll.substr(0,i),prop,value, TVAL)) return RET; coll=coll.substr(i+1); i=coll.find(","); }
template<class T>
bool plp_exists(const vector<T>& coll, string prop, string value, string xxx=""){
	for(size_t i=0;i<coll.size();i++) if(coll[i].vars.at(prop)==value) return true; return false;
}
bool plp_exists(const PLP& plp, string coll, string prop, string value, string xxx=""){
#	define FUNC plp_exists
#	define RET true
#	define TVAL ""
	SPLIT_COLL
	CALL(variables);
	CALL(parameters);
	CALL(resources);
	CALL(preconditions);
	CALL(concurent_conditions);
	CALL(concurent_modules);
	CALL(goals);
	CALL(side_effects);
	CALL(failure_modes);
	return false;
#	undef FUNC
#	undef TVAL
#	undef RET
}
template<class T>
string plp_find(const vector<T>& coll, string prop, string value, string tprop){
	static T t;
	for(size_t i=0;i<coll.size();i++)
		if(
			coll[i].vars.find(prop)!=coll[i].vars.end() and
			coll[i].vars.at(prop)==value
		){
			if(coll[i].vars.find(tprop)==coll[i].vars.end())
				return string()+"ERROR[property_"+tprop+"_isn't_found_on_object."+prop+"="+value+"]";
			return coll[i].vars.at(tprop);
		}
	return string()+"ERROR[object."+prop+"="+value+"_isn't_found]";
}

string plp_find(const PLP& plp, string coll, string prop, string value, string tprop){
#	define FUNC plp_exists
#	define TVAL tprop
#	define RET plp_find(plp,coll.substr(0,i),prop,value, tprop)
	SPLIT_COLL
#	undef FUNC
#	define FUNC plp_find
	CALL(variables);
	CALL(parameters);
	CALL(resources);
	CALL(preconditions);
	CALL(concurent_conditions);
	CALL(concurent_modules);
	CALL(goals);
	CALL(side_effects);
	CALL(failure_modes);
	return string()+"ERROR[Collection_"+coll+"_isn't_found]";
#	undef FUNC
#	undef TVAL
#	undef RET
}

Script predicate_to_variable(string var_name, Script& scr){
	Script res;
	res.properties = scr.properties;
	vector<string> ass = scr.assignments();
	BOOST_FOREACH(string line, ass){
		//cout<<"ASS: "<<line<<endl;
		res.lines.push_back(line);
	}
	vector<string> comp = scr.comparisons();
	string all_comp = var_name + " = (";
	BOOST_FOREACH(string line, comp){
		//cout<<"COM: "<<line<<endl;
		all_comp += "("+line+")" + " and ";
	}
	all_comp += "True)";
	res.lines.push_back(all_comp);

	return res;
}


namespace for_time_to_seconds{
	bool isNumeric(const std::string& s){
		int st=0;
		if(s[st]=='+' or s[st]=='-') st=1;
		bool n=false;
		for(size_t i=st;i<s.size();i++){ if( s[i]<'0' or '9'<s[i] ) return false; n=true; }
		return n;
	}
}
string time_to_seconds(string src){
	using namespace for_time_to_seconds;
	boost::trim(src);
	if(isNumeric(src)) return src;
	size_t i = string::npos; double sec;
	#define BLOCK_BNG do{
	#define BLOCK_END }while(false);
	#define CHECK(N,P) i = src.find(N); if(i!=string::npos){ string substr=src.substr(0,i); stringstream s; boost::trim(substr); s<<substr; s>>sec; sec*=P; break; }
	BLOCK_BNG
		CHECK("millisec",0.001)
		CHECK("min",60)
		CHECK("hour",(60*60))
	BLOCK_END
	#undef CHECK
	#undef BLOCK_BNG
	#undef BLOCK_END
	stringstream ret; ret<<sec;
	return ret.str();
}


#undef CALL

#define PLP_exists( SET, PROP, VALUE, RET ) { RET=false; for(size_t i=0;i<plp.SET.size();i++) if(plp.SET[i].vars[#PROP]==VALUE){ RET=true; break; } }
#define PLP_find( SET, PROP, VALUE, RET ) { for(size_t i=0;i<plp.SET.size();i++) if(plp.SET[i].vars[#PROP]==VALUE){ RET=plp.SET[i]; break; } }
#define PLP_get( PROP, OBJ, RET ) { RET = OBJ.PROP; }
#define GET_GLOBAL_VAR(V) (string()<<"get_global_var_float(\""<<V<<"\")")
#define SET_GLOBAL_VAR(V, D) (string()<<"set_global_var_float(\""<<V<<"\","<<D<<")")

#define CHECK_VALUE(var,source) (var.source==""?std::string()+"UNKNOWN_"#source"_OF_"+var.name:var.source)
#define PREDICATE(X) res << ( string() << X )

#define VARIABLES_SET "variables,parameters,resources"

//============= SINGLE TRANSForMATIONS ===================================================

Script transform(const PLP& plp, const Variable& var, int& error_code){
	Script res;
	string source_name = string()+var.name+"_source";
	bool pl=isnum(var.lower), pu=isnum(var.upper);
	if(pl or pu){
		PREDICATE( source_name <<" = {" <<CHECK_VALUE(var,source)<<"}" );
	}
	if(pl){
		PREDICATE( var.lower << " <= "+source_name );
	}
	if(pu){
		PREDICATE( source_name<<" <= " << var.upper );
	}
	return res;
}

Script transform(const PLP& plp, const Parameter& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	bool pl=isnum(par.lower), pu=isnum(par.upper),pf=isnum(par.frequency);
	if( (pl and pu) or pf ){
		PREDICATE( source_name <<" = {" <<CHECK_VALUE(par,source)<<"}" );
	}
	if( pl and pu  ){
		PREDICATE( par.lower << " <= "+source_name<<" and "<<source_name<<" <= " << par.upper );
	}
	if( pf ){
		PREDICATE( "_hz = " << "hz("<<source_name<<")" );
		PREDICATE( "_hz >= " << par.frequency );
	}
	return res;
}

template<class T>
string name_of_gloval_var(string pref,const PLP& plp, const T& p){ return string()<<pref<<plp.name<<"_"<<p.name; }

Script transform_preconditions(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	PREDICATE( source_name <<" = {" <<CHECK_VALUE(par,source)<<"}" );
	PREDICATE( SET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par), source_name) );
	if( isnum(par.minimal_initial_value) ){
		PREDICATE( par.minimal_initial_value << " <= "<<source_name );
	}
	return res;
}
Script transform_consumption(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	bool m=isnum(par.maximal_consumption),c=isnum(par.consumption_speed);
	if( m or c ){
		PREDICATE( source_name <<" = {"<<CHECK_VALUE(par,source)<<"}" );
	}
	if( m ){
		PREDICATE( "_isd = " << GET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par)) );
		PREDICATE( "_isd" <<" - " << par.maximal_consumption << " < " <<source_name );
	}
	if( c ){
		PREDICATE( "_der = " << "der( "<< source_name <<" , 1000)" );
		PREDICATE( "_der" << " <= " <<par.consumption_speed );
	}
	return res;
}
Script transform_effects(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	if( isnum(par.maximal_consumption) ){
		PREDICATE( source_name << " = {"<<CHECK_VALUE(par,source)<<"}" );
		PREDICATE( "_isd = " << GET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par)) );
		PREDICATE( "_isd" <<" - " << source_name  << " <= " <<par.maximal_consumption );
	}
	return res;
}

Script transform(const PLP& plp, const Precondition& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_"+par.parameter+"_source";
	if(plp_exists(plp, VARIABLES_SET, "name", par.parameter)){
		bool mi=isnum(par.minimal_value), ma=isnum(par.max_value);
		if(mi or ma){
			PREDICATE( source_name << " = {"<<plp_find(plp, VARIABLES_SET, "name", par.parameter, "source")<<"}" );
		}
		if( mi ){
			PREDICATE( par.minimal_value <<" <= "<< source_name);
		}
		if( ma ){
			PREDICATE( par.max_value <<" >= "<< source_name);
		}
	}
	return res;
}

Script transform(const PLP& plp, const ConcurentCondition& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_"+par.parameter+"_source";
	if(plp_exists(plp, VARIABLES_SET, "name", par.parameter)){
		bool mi=isnum(par.minimal_value), ma=isnum(par.max_value);
		if(mi or ma){
			PREDICATE( source_name << " = {"<<plp_find(plp, VARIABLES_SET, "name", par.parameter, "source")<<"}" );
		}
		if( mi ){
			PREDICATE( par.minimal_value <<" <= "<< source_name);
		}
		if( ma ){
			PREDICATE( par.max_value <<" >= "<< source_name);
		}
	}
	return res;
}

Script transform(const PLP& plp, const ConcurentModule& par, int& error_code){
	Script res;
	string state_name = "_state_" + par.name;
	PREDICATE( state_name << " = " <<  "get_module_status( '" << par.name <<"' )");
	if(par.state == "Never together"){
		PREDICATE( state_name << " == 'stop' ");
	}else{
		PREDICATE( state_name << " == 'run' ");
	}
	return res;
}

Script transform(const PLP& plp, const SideEffect& par, int& error_code){
	Script res;
	string source_name = string()+"side_effect"+"_"+par.parameter+"_source";
	if(plp_exists(plp, VARIABLES_SET, "name", par.parameter)){
		bool mi=isnum(par.minimal_value), ma=isnum(par.max_value);
		if(mi or ma){
			PREDICATE( source_name << " = {"<<plp_find(plp, VARIABLES_SET, "name", par.parameter, "source")<<"}" );
		}
		if( mi ){
			PREDICATE( par.minimal_value <<" <= "<< source_name);
		}
		if( ma ){
			PREDICATE( par.max_value <<" >= "<< source_name);
		}
	}
	return res;
}

Script transform_start(const PLP& plp, const Goal& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	PREDICATE( "_now = Now()");
	PREDICATE( SET_GLOBAL_VAR(name_of_gloval_var("StartTime_",plp,par), "_now") );
	return res;
}
Script transform_goal(const PLP& plp, const Goal& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	if(plp_exists(plp, VARIABLES_SET, "name", par.parameter)) {
		bool mi=isnum(par.minimal_value), ma=isnum(par.max_value);
		if(mi or ma){
			PREDICATE( source_name << " = {"<<plp_find(plp, VARIABLES_SET, "name", par.parameter, "source")<<"}" );
		}
		if( mi ){
			PREDICATE( par.minimal_value <<" <= "<< source_name);
		}
		if( ma ){
			PREDICATE( par.max_value <<" >= "<< source_name);
		}
	}
	return res;
}
Script transform_time(const PLP& plp, const Goal& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	if(plp_exists(plp, VARIABLES_SET, "name", par.parameter) and isnum(par.max_time_to_complete)){
		PREDICATE( "_now = Now()");
		PREDICATE( "_st = " << GET_GLOBAL_VAR(name_of_gloval_var("StartTime_",plp,par)) );
		PREDICATE( "_duration = _now - _st");
		Script tmp = transform_goal(plp,par,error_code);
		if(error_code) return res;
		res << predicate_to_variable( "_goal",tmp );
		PREDICATE( "_goal or ( not( _goal ) and _duration < " + time_to_seconds(par.max_time_to_complete) << " )" );
	}
	return res;
}


//=============== COLLECTIONS OF TRANSForMATIONS ========================================

TRANSForM_COLLECTION(Variable)
TRANSForM_COLLECTION(Parameter)

ScriptCollection transform(const PLP& plp, const vector<Resource>& coll, int& error_code){
	ScriptCollection res_coll;
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_precondition");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_start");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_preconditions(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_consumption");
		res.properties<<(string()<<"module "<<plp.name);
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_consumption(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_effects");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_stop");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_effects(plp, coll[i], error_code);
		res_coll<<res;
	}
	return res_coll;
}

TRANSForM_COLLECTION(Precondition)
TRANSForM_COLLECTION(ConcurentCondition)
TRANSForM_COLLECTION(ConcurentModule)
TRANSForM_COLLECTION(SideEffect)


ScriptCollection transform(const PLP& plp, const vector<Goal>& coll, int& error_code){
	ScriptCollection res_coll;
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Goal_start");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_start");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_start(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Goal_time_check");
		res.properties<<(string()<<"module "<<plp.name);
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_time(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicate");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Goal_stop");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_stop");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_goal(plp, coll[i], error_code);
		res_coll<<res;
	}
	return res_coll;
}

//============================================================================


ScriptCollection transform(const PLP& plp, int& error_code){
	ScriptCollection res;
	res << transform(plp, plp.variables, error_code); 					if(error_code) return res;
	res << transform(plp, plp.parameters, error_code); 					if(error_code) return res;
	res << transform(plp, plp.resources, error_code); 					if(error_code) return res;
	res << transform(plp, plp.preconditions, error_code); 				if(error_code) return res;
	res << transform(plp, plp.concurent_conditions, error_code); 		if(error_code) return res;
	res << transform(plp, plp.concurent_modules, error_code); 			if(error_code) return res;
	res << transform(plp, plp.side_effects, error_code); 				if(error_code) return res;
	res << transform(plp, plp.goals, error_code); 						if(error_code) return res;
	return res;
}



vector<MonitorningScript> PLPCompiler::compile(const PLP& plp, int& error_code){
	vector<MonitorningScript> results;

	results << transform(plp, error_code);

	return results;
}





