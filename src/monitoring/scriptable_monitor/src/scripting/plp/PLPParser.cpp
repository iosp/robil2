#include "PLPParser.h"

#include <iostream>
#include <fstream>

#include "PLPDataStruct.h"
#include "ParsingTools.h"

using namespace std;


PLPParser::PLPParser(std::istream& script_stream)
:
		_script_stream(script_stream),
		_plp(0)
{

}

PLPParser::~PLPParser(){
	setPLP(NULL);
}
void PLPParser::setPLP(PLP* p){
	if(_plp)delete _plp; _plp=p;
}

bool is_delimiter(char c){
	if('a'<=c and c<='z') return false;
	if('A'<=c and c<='Z') return false;
	if('0'<=c and c<='9') return false;
	if(c=='_' or c=='-' or c=='/') return false;
	return true;
}
bool isNumeric(const std::string& s){
	int st=0;
	if(s[st]=='+' or s[st]=='-') st=1;
	bool n=false;
	for(size_t i=st;i<s.size();i++){ if( s[i]<'0' or '9'<s[i] ) return false; n=true; }
	return n;
}

vector<Token> tokenizer(istream& text){
	vector<Token> tokens;
	char c;
	int index, line, pose;
	index=0; line=pose=1;
	std::stringstream word;
	Token token (std::string(""), line, pose, index );
	while(not text.eof()){
		text.read(&c,1); index++;
		if(c=='\n'){
			line++; pose=0;
		}else{
			pose++;
		}
		bool is_delim = is_delimiter(c);
		if( is_delim or token.delimiter ){
			token.name=word.str();
			tokens.push_back(token);
			word.str("");
			token.reset(line, pose, index, is_delim);
		}
		word<<c;
	}
	token.name=word.str();
	tokens.push_back(token);
	token.name="\n";
	tokens.push_back(token);
	return tokens;
}


vector<Token> filter_numbers(const vector<Token>& tokens){
	vector<Token> _res,_tmp;
	InTokenStream res(_res);
	OutTokenStream inp(tokens);
	int state=0;
	while(inp.eof()==false){
		Token t; inp>>t;
		if(state==0){
			if(isNumeric(t.name)){
				_tmp.push_back(t);
				state=1;
			}else if(t.name=="-" or t.name=="+"){
				_tmp.push_back(t);
				state=2;
			}else if(t.name=="."){
				_tmp.push_back(t);
				state=3;
			}else{
				res<<t;
			}
		}else if(state==1){
			if(t.name=="."){
				_tmp.push_back(t);
				state=3;
			}else if(t.delimiter){
				Token n = combine(_tmp); n.isNumber=true;
				res<< n <<t;
				_tmp.clear();
				state=0;
			}else{
				copy(res, _tmp);
				res<<t;
				_tmp.clear();
				state=0;
			}
		}else if(state==2){
			if(isNumeric(t.name)){
				_tmp.push_back(t);
				state=1;
			}else{
				copy(res, _tmp);
				res<<t;
				_tmp.clear();
				state=0;
			}
		}else if(state==3){
			if(isNumeric(t.name)){
				_tmp.push_back(t);
				Token n = combine(_tmp); n.isNumber=true;
				res<< n;
				_tmp.clear();
				state=0;
			}else{
				Token tt = _tmp.back();
				_tmp.erase(_tmp.end()-1);
				if(_tmp.size()>0){
					Token n = combine(_tmp); n.isNumber=true;
					res<< n <<tt<<t;
				}
				else res<<tt<<t;
				_tmp.clear();
				state=0;
			}
		}
	}
	return _res;
}

vector<Token> filter_delimiters(const vector<Token>& tokens){
	vector<Token> _res,_tmp;
	InTokenStream res(_res);
	OutTokenStream inp(tokens);
	while(inp.eof()==false){
		Token t; inp>>t;
		if(t.delimiter){
			if(t.name==":") res<<t;
			else if(t.name=="\n"){
				if(_res.back().name!=" " or true){
					Token tt=t; tt.name=" "; res<<tt;
				}
			}
		}else{
			res<<t;
		}
	}
	return _res;
}

void PLPParser::read_all(){
	_tokens = tokenizer(_script_stream);
	_tokens = filter_numbers(_tokens);
	_tokens = filter_delimiters(_tokens);
}



bool parse_item_name(std::string& name, OutTokenStream& tokens){
	Token t;
	vector<Token> name_of;
	while(tokens.eof()==false){
		tokens>>t;
		if(t.name==" "){
			if(name_of.empty()){
				return false;
			}else{
				Token tt = combine(name_of," ");
				name = tt.name;
				return true;
			}
		}else{
			name_of.push_back(t);
		}
	}
	return false;
}

bool parse_plp_params_type(PLP& plp, OutTokenStream& tokens){
	Token t;
	READ(t, "Type")
	READ(t, ":")
	if(parse_item_name(plp.type, tokens)) return true;
	return false;
}
bool parse_plp_params_goal_repeat(PLP& plp, OutTokenStream& tokens){
	Token t;
	READ(t, "Goal")
	READ(t, "repeat")
	READ(t, ":")
	READ_ANY(t)
	string tp = t.name;
	READ(t, " ");
	set(plp , goal_repeat , tp);
	return true;
}
bool parse_plp_params_repeat_freq(PLP& plp, OutTokenStream& tokens){
	Token t;
	READ(t, "Repeat")
	READ_ANY(t) if(t.name!="frequencey" and t.name!="frequency"){tokens.set_error(__LINE__); return false;}
	READ(t, ":")
	READ_VALUE(t)
	set(plp, repeat_frequency, t.name);
	return true;
}
bool parse_plp_params_variables(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Variables")
	COLLECTION_ITEM_NAME("Name", variables, Variable, name )
	COLLECTION_ITEM_REQ("Type",type)
	COLLECTION_ITEM_REQ("Source",source)
	COLLECTION_ITEM_OPT("Lower bound", lower)
	COLLECTION_ITEM_OPT("Upper bound", upper)
	COLLECTION_END
}
bool parse_plp_params_parameters(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Parameters")
	COLLECTION_ITEM_NAME("Name", parameters, Parameter, name )
	COLLECTION_ITEM_REQ("Type",type)
	COLLECTION_ITEM_REQ("Source",source)
	COLLECTION_ITEM_OPT("Lower bound", lower)
	COLLECTION_ITEM_OPT("Upper bound", upper)
	COLLECTION_ITEM_OPT("Frequency", frequency)
	COLLECTION_END
}
bool parse_plp_params_resources(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Resources")
	COLLECTION_ITEM_NAME("Resource", resources, Resource, name )
	COLLECTION_ITEM_REQ("Type",type)
	COLLECTION_ITEM_REQ("Source",source)
	COLLECTION_ITEM_OPT("Minimal initial value", minimal_initial_value)
	COLLECTION_ITEM_OPT("Maximal consumption", maximal_consumption)
	COLLECTION_ITEM_OPT("Consumption speed", consumption_speed)
	COLLECTION_END
}

bool parse_plp_params_precondition(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Preconditions")
	COLLECTION_ITEM_NAME("Precondition name", preconditions, Precondition, name )
	COLLECTION_ITEM_REQ("Parameter", parameter)
	COLLECTION_ITEM_OPT("Minimal value", minimal_value)
	COLLECTION_ITEM_OPT("Max value", max_value)
	COLLECTION_END
}
bool parse_plp_params_conc_condition(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Concurrent conditions")
	COLLECTION_ITEM_NAME("Condition name", concurent_conditions, ConcurentCondition, name )
	COLLECTION_ITEM_OPT("Parameter/Variable", parameter)
	COLLECTION_ITEM_OPT("Minimal value", minimal_value)
	COLLECTION_ITEM_OPT("Max value", max_value)
	COLLECTION_END
}
bool parse_plp_params_conc_modules(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Concurrent modules")
	COLLECTION_ITEM_NAME("Name", concurent_modules, ConcurentModule, name )
	COLLECTION_ITEM_REQ("State", state)
	COLLECTION_END
}
bool parse_plp_params_goals(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Goals")
	COLLECTION_ITEM_NAME("Goal name", goals, Goal, name )
	COLLECTION_ITEM_REQ("Goal parameter", parameter)
	COLLECTION_ITEM_OPT("Minimal value", minimal_value)
	COLLECTION_ITEM_OPT("Max value", max_value)
	COLLECTION_ITEM_OPT("Success probability", success_probability)
	COLLECTION_ITEM_OPT("Max time to complete", max_time_to_complete)
	COLLECTION_END
}
bool parse_plp_params_sideeffects(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Side effects")
	COLLECTION_ITEM_NAME("Parameter", side_effects, SideEffect, parameter )
	COLLECTION_ITEM_OPT("Minimal value", minimal_value)
	COLLECTION_ITEM_OPT("Max value", max_value)
	COLLECTION_ITEM_OPT("Probability", probability)
	COLLECTION_END
}
bool parse_plp_params_failers(PLP& plp, OutTokenStream& tokens, int& stage){
	COLLECTION_READ_NAME("Failure modes")
	COLLECTION_ITEM_NAME("Name", failure_modes, FailureMode, name )
	COLLECTION_ITEM_OPT("Severity", severity)
	COLLECTION_ITEM_OPT("Error code", error_code)
	COLLECTION_ITEM_OPT("Failure probability", failure_probability)
	COLLECTION_END
}


bool parse_plp_params(PLP& plp, OutTokenStream& tokens){
	Token t;
	bool res = parse_plp_params_type(plp,tokens);
	if(not res) return false;
	while(not tokens.eof()){
		tokens.push();
		READ_ANY(t)
		if( t.name=="Goal"){
			READ_ANY(t)
			if( t.name=="repeat"){
				tokens.pop();
				tokens.push();
				res = parse_plp_params_goal_repeat(plp,tokens);
				if(not res){ tokens.drop(); return false; }else{ tokens.pop(); tokens>>t; }
			}else{ tokens.pop(); tokens>>t;	}
		}else if( t.name=="Repeat"){
			READ_ANY(t)
			if( t.name=="frequencey" or t.name=="frequency"){
				tokens.pop();
				tokens.push();
				res = parse_plp_params_repeat_freq(plp,tokens);
				if(not res){ tokens.drop(); return false; }else{ tokens.pop(); tokens>>t; }
			}else{ tokens.pop(); tokens>>t;	}
		}else if( t.name=="Variables"){
			tokens.pop();
			tokens.push();
			int stage=0;
			res = parse_plp_params_variables(plp,tokens,stage);
			if(res) tokens.drop(); else{
				if(stage==0){ tokens.pop(); tokens>>t; }
				else{ tokens.drop(); return false; }
			}
		}else if( t.name=="Parameters"){
			tokens.pop();
			tokens.push();
			int stage=0;
			res = parse_plp_params_parameters(plp,tokens,stage);
			if(res) tokens.drop(); else{
				if(stage==0){ tokens.pop(); tokens>>t; }
				else{ tokens.drop(); return false; }
			}
		}else if( t.name=="Resources"){
			tokens.pop();
			tokens.push();
			int stage=0;
			res = parse_plp_params_resources(plp,tokens,stage);
			if(res) tokens.drop(); else{
				if(stage==0){ tokens.pop(); tokens>>t; }
				else{ tokens.drop(); return false; }
			}
		}else if( t.name=="Preconditions"){
			tokens.pop();
			tokens.push();
			int stage=0;
			res = parse_plp_params_precondition(plp,tokens,stage);
			if(res) tokens.drop(); else{
				if(stage==0){ tokens.pop(); tokens>>t; }
				else{ tokens.drop(); return false; }
			}
		}else if( t.name=="Goals"){
			tokens.pop();
			tokens.push();
			int stage=0;
			res = parse_plp_params_goals(plp,tokens,stage);
			if(res) tokens.drop(); else{
				if(stage==0){ tokens.pop(); tokens>>t; }
				else{ tokens.drop(); return false; }
			}
		}else if( t.name=="Side"){
			READ_ANY(t)
			if( t.name=="effects"){
				tokens.pop();
				tokens.push();
				int stage=0;
				res = parse_plp_params_sideeffects(plp,tokens,stage);
				if(res) tokens.drop(); else{
					if(stage==0){ tokens.pop(); tokens>>t; }
					else{ tokens.drop(); return false; }
				}
			}else{ tokens.pop(); tokens>>t; }
		}else if( t.name=="Failure"){
			READ_ANY(t)
			if( t.name=="modes"){
				tokens.pop();
				tokens.push();
				int stage=0;
				res = parse_plp_params_failers(plp,tokens,stage);
				if(res) tokens.drop(); else{
					if(stage==0){ tokens.pop(); tokens>>t; }
					else{ tokens.drop(); return false; }
				}
			}else{ tokens.pop(); tokens>>t; }
		}else if( t.name=="Concurrent"){
			READ_ANY(t)
			if( t.name=="conditions"){
				tokens.pop();
				tokens.push();
				int stage=0;
				res = parse_plp_params_conc_condition(plp,tokens,stage);
				if(res) tokens.drop(); else{
					if(stage==0){ tokens.pop(); tokens>>t; }
					else{ tokens.drop(); return false; }
				}
			}else if( t.name=="modules"){
				tokens.pop();
				tokens.push();
				int stage=0;
				res = parse_plp_params_conc_modules(plp,tokens,stage);
				if(res) tokens.drop(); else{
					if(stage==0){ tokens.pop(); tokens>>t; }
					else{ tokens.drop(); return false; }
				}
			}else{
				tokens.pop(); tokens>>t;
			}
		}
		else tokens.drop();
	}
	return true;
}

bool parse_plp(PLP& plp, OutTokenStream& tokens){
	Token t;
	if(tokens.eof()==false) tokens>>t;
	if(t.name==":"){
		if(parse_item_name(plp.name, tokens)){
			if(parse_plp_params(plp, tokens)){
				return true;
			}
		}else return false;
	}
	return false;
}

bool PLPParser::create_structures(){
	PLP* plp_ptr = new PLP(); PLP& plp = *plp_ptr;
	OutTokenStream tokens(_tokens);
	while(tokens.eof()==false){
		Token t; tokens>>t;
		if(t.name=="PLP"){
			bool res = parse_plp(plp, tokens);
			if(not res){
				delete plp_ptr;
				t = tokens.error();
				cerr<<"Parsing error ["<<t.line<<":"<<t.pose<<"]. PLP script is corrupted. parser line: "<<tokens.line<<endl;
				return false;
			}
			break;
		}
	}
	setPLP(plp_ptr);
	return true;
}


