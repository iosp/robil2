/*
 * PLPCompiler.cpp
 *
 *  Created on: May 7, 2014
 *      Author: dan
 */

#include "PLPCompiler.h"

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

#define TRANSFORM_COLLECTION(ELEMENT)\
	Script transform(const PLP& plp, const vector<ELEMENT>& ELEMENT_##_collection, int& error_code){\
		Script res;\
		res.properties<<(string()<<"type predicates");\
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

#undef CALL

#define PLP_exists( SET, PROP, VALUE, RET ) { RET=false; for(size_t i=0;i<plp.SET.size();i++) if(plp.SET[i].vars[#PROP]==VALUE){ RET=true; break; } }
#define PLP_find( SET, PROP, VALUE, RET ) { for(size_t i=0;i<plp.SET.size();i++) if(plp.SET[i].vars[#PROP]==VALUE){ RET=plp.SET[i]; break; } }
#define PLP_get( PROP, OBJ, RET ) { RET = OBJ.PROP; }
#define GET_GLOBAL_VAR(V) (string()<<"get_global_var(\""<<V<<"\")")
#define SET_GLOBAL_VAR(V, D) (string()<<"set_global_var(\""<<V<<"\","<<D<<")")

#define CHECK_VALUE(var,source) (var.source==""?std::string()+"UNKNOWN_"#source"_OF_"+var.name:var.source)
#define PREDICAT(X) res << ( string() << X )

Script transform(const PLP& plp, const Variable& var, int& error_code){
	Script res;
	string source_name = string()+var.name+"_source";
	res << ( string() << source_name <<" = {" <<CHECK_VALUE(var,source)<<"}" );
	res << ( string() << var.lower << " <= "+source_name<<" and "<<source_name<<" <= " << var.upper );
	return res;
}

Script transform(const PLP& plp, const Parameter& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	bool pl=isnum(par.lower), pu=isnum(par.upper),pf=isnum(par.frequency);
	if( (pl and pu) or pf ){
		res << ( string() << source_name <<" = {" <<CHECK_VALUE(par,source)<<"}" );
	}
	if( pl and pu  ){
		res << ( string() << par.lower << " <= "+source_name<<" and "<<source_name<<" <= " << par.upper );
	}
	if( pf ){
		res << ( string() << "hz("<<source_name<<") >= " << par.frequency );
	}
	return res;
}

template<class T>
string name_of_gloval_var(string pref,const PLP& plp, const T& p){ return string()<<pref<<plp.name<<"_"<<p.name; }

Script transform_preconditions(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	res << ( string() << source_name <<" = {" <<CHECK_VALUE(par,source)<<"}" );
	res << SET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par), source_name);
	if( isnum(par.minimal_initial_value) ){
		res << ( string() << par.minimal_initial_value << " <= "<<source_name );
	}
	return res;
}
Script transform_consumption(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	bool m=isnum(par.maximal_consumption),c=isnum(par.consumption_speed);
	if( m or c ){
		res << ( string() << source_name <<" = {"<<CHECK_VALUE(par,source)<<"}" );
	}
	if( m ){
		res << ( string() << GET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par)) <<" - " << par.maximal_consumption << " < " <<source_name );
	}
	if( c ){
		res << ( string() << "der( "<< source_name <<" , 1000)" << " <= " <<par.consumption_speed );
	}
	return res;
}
Script transform_effects(const PLP& plp, const Resource& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_source";
	if( isnum(par.maximal_consumption) ){
		res << ( string() << source_name << " = {"<<CHECK_VALUE(par,source)<<"}" );
		res << ( string() << GET_GLOBAL_VAR(name_of_gloval_var("InitSourceData_",plp,par)) <<" - " << source_name  << " <= " <<par.maximal_consumption );
	}
	return res;
}

Script transform(const PLP& plp, const Precondition& par, int& error_code){
	Script res;
	string source_name = string()+par.name+"_"+par.parameter+"_source";
	if(plp_exists(plp, "variables,parameters", "name", par.parameter)){
		bool mi=isnum(par.minimal_value), ma=isnum(par.max_value);
		if(mi or ma){
			res << ( string() << source_name << " = {"<<plp_find(plp, "variables,parameters", "name", par.parameter, "source")<<"}" );
		}
		if( mi ){
			PREDICAT( par.minimal_value <<" <= "<< source_name);
		}
		if( ma ){
			PREDICAT( par.max_value <<" >= "<< source_name);
		}
	}
	return res;
}

TRANSFORM_COLLECTION(Variable)
TRANSFORM_COLLECTION(Parameter)

ScriptCollection transform(const PLP& plp, const vector<Resource>& coll, int& error_code){
	ScriptCollection res_coll;
	{
		Script res;
		res.properties<<(string()<<"type predicates");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_precondition");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_start");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_preconditions(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicates");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_consumption");
		res.properties<<(string()<<"module "<<plp.name);
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_consumption(plp, coll[i], error_code);
		res_coll<<res;
	}
	{
		Script res;
		res.properties<<(string()<<"type predicates");
		res.properties<<(string()<<"name "<<plp.name<<"_"<<"Resource_effects");
		res.properties<<(string()<<"module "<<plp.name);
		res.properties<<(string()<<"time on_stop");
		for(size_t i=0;i<coll.size() and not error_code;i++)
			res << transform_effects(plp, coll[i], error_code);
		res_coll<<res;
	}
	return res_coll;
}

TRANSFORM_COLLECTION(Precondition)



ScriptCollection transform(const PLP& plp, int& error_code){
	ScriptCollection res;
	res << transform(plp, plp.variables, error_code);
	res << transform(plp, plp.parameters, error_code);
	res << transform(plp, plp.resources, error_code);
	res << transform(plp, plp.preconditions, error_code);
	return res;
}



vector<MonitorningScript> PLPCompiler::compile(const PLP& plp, int& error_code){
	vector<MonitorningScript> results;

	results << transform(plp, error_code);

	return results;
}


vector<MonitorningScript> PLPCompiler::compile_testing(const PLP& plp, int& error_code){
	vector<MonitorningScript> results;
	MonitorningScript s1;
	s1.properties<<"type predicate";
	s1.properties<<"name predicate_script1";
	s1.properties<<"interval 2";
	s1<<"core1 = {/scan/ranges[0:1]}";
	s1<<"core2 = {/scan/ranges[1:2]}";
	s1<<"core3 = {/scan/ranges[2:3]}";
	s1<<"core4 = {/scan/ranges[3:4]}";
	s1<<"cpu_th = {/scan/ranges[4:5]}";
	s1<<"core_th = {/scan/ranges[5:6]}";
	s1<<"average = SomeFunction(core1, core2, core3, core4)";
	s1<<"average < cpu_th";
	s1<<"core1 < core_th";
	s1<<"core2 < core_th";
	s1<<"core3 > core_th";
	s1<<"core4 < core_th";

	MonitorningScript s2;
	s2.properties<<"type predicate";
	s2.properties<<"name predicate_script2";
	s2.properties<<"interval 3";
	s2<<"core1 = {/scan/ranges[0:1]}";
	s2<<"core2 = {/scan/ranges[1:2]}";
	s2<<"core3 = {/scan/ranges[2:3]}";
	s2<<"core4 = {/scan/ranges[3:4]}";
	s2<<"cpu_th = {/scan/ranges[4:5]}";
	s2<<"core_th = {/scan/ranges[5:6]}";
	s2<<"average = SomeFunction(core1, core2, core3, core4)";
	s2<<"average < cpu_th";
	s2<<"core1 < core_th";
	s2<<"core2 < core_th";
	s2<<"core3 > core_th";
	s2<<"core4 < core_th";

	results << s1 << s2;

	return results;
}



