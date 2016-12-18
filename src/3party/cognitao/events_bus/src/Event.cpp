/*
 * Event.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: dan
 */

#include <cognitao/bus/Event.h>


namespace cognitao{
namespace bus{

const
Event Event::IDLE = Event(
		  Event::name_t("IDLE")
		, Event::channel_t("SPECIAL_EVENT_TYPE")
		, Event::context_t("")
		, Event::SCOPE_GLOBAL
		, Event::stamp_t()
		, Event::source_t("")
		, true
		);

Event::Event()
	: _name("")
	, _channel("empty_event")
	, _context("")
	, _scope(SCOPE_GLOBAL)
	, _stamp()
	, _source()
	, _internal(true)
	, _parameters()
{

}

class Exception_SyntaxError: public std::exception{};
class Exception_NameIsEmpty: public std::exception{};


Event::Event(
	  const name_t& 	name_
)
	: _name		(name_)
	, _channel	("")
	, _context	("")
	, _scope	(SCOPE_GLOBAL)
	, _stamp	()
	, _source	("")
	, _internal	(false)
	, _parameters()
{
	if(_name.is_template()) return;
	parse(_name.text);
}

Event::Event(
	  const string& 	name_
)
	: _name		(name_)
	, _channel	("")
	, _context	("")
	, _scope	(SCOPE_GLOBAL)
	, _stamp	()
	, _source	("")
	, _internal	(false)
	, _parameters()
{
	if(_name.is_template()) return;
	parse(_name.text);
}

Event::Event(
	  const name_t& 	  name_
	, const channel_t& 	  channel_
	, const context_t& 	  context_
	, const scope_t& 	  scope_
	, const stamp_t& 	  stamp_
	, const source_t& 	  source_
	, const internal_t&   internal_
	, const parameter_t&  params_
)
	: _name		 (name_)
	, _channel	 (channel_)
	, _context	 (context_)
	, _scope	 (scope_)
	, _stamp	 (stamp_)
	, _source	 (source_)
	, _internal	 (internal_)
	, _parameters(params_)
{
	if(_name.text.size()>0 and _name.text[0]!='/') _name.text = string("/")+_name.text;
	if(_context.text.size()>0 and _context.text[0]!='/') _context.text = string("/")+_context.text;
	if( HierarchicalName(full_name()).is_template() ) *this = Event( full_name() );
}

Event::Event(
	const Event& event
)
	: _name		 (event._name)
	, _channel	 (event._channel)
	, _context	 (event._context)
	, _scope	 (event._scope)
	, _stamp	 (event._stamp)
	, _source	 (event._source)
	, _internal	 (event._internal)
	, _parameters(event._parameters)
{

}

const Event& Event::operator=(const Event& event)
{
	  _name		 =(event._name)
	, _channel	 =(event._channel)
	, _context	 =(event._context)
	, _scope	 =(event._scope)
	, _stamp	 =(event._stamp)
	, _source	 =(event._source)
	, _internal	 =(event._internal)
	, _parameters=(event._parameters)
	;

	return event;
}

Event::Event(
	  const Event& event
	, const channel_t& channel_
	, const context_t& context_
)
	: _name		 (event._name)
	, _channel	 (channel_)
	, _context	 (context_)
	, _scope	 (event._scope)
	, _stamp	 (event._stamp)
	, _source	 (event._source)
	, _internal	 (event._internal)
	, _parameters(event._parameters)
{

}

string Event::parameters_str()const
{
	if(_parameters.size() ==0 ) return "";
	std::stringstream s;
	s<<"(";
	//for(size_t i=0;i<_parameters.size()-1;i++) s<<_parameters[i]<<",";
	//if(_parameters.size()>0) s<<_parameters.back();
	s<<_parameters;
	s<<")";
	return s.str();
}

string Event::full_name( bool with_params )const
{
	string scope_str="";
	if(_scope == SCOPE_DOWN) scope_str = "_";
	else if(_scope == SCOPE_UP) scope_str = "^";
	else if(_scope == SCOPE_SPOT) scope_str = "=";
	else if(_scope == SCOPE_CHILD) scope_str = "__";
	else if(_scope == SCOPE_PARENT) scope_str = "^^";

	string params = "";
	if(with_params and parameters().size()>0) params = parameters_str();
	return
			_context.str() + "/" + _channel + "." + scope_str + _name.str() + params;
}

void Event::parse(const string& text)
{
	size_t di1 = _name.text.find(".");
	size_t di2 = _name.text.find("!");
	size_t di3 = _name.text.find("?");

	int di_status = int(di1!=string::npos)+int(di2!=string::npos)+int(di3!=string::npos);
	if( di_status > 1 )
	{
		std::cerr << "[e] more then one direction symbols detected in event full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
		throw Exception_SyntaxError();
	}

	size_t di = (di1!=string::npos?di1:0)+(di2!=string::npos?di2:0)+(di3!=string::npos?di3:0);

	if( di == text.size()-1 )
	{
		std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
		throw Exception_SyntaxError();
	}

	string pref;
	string suff;
	if( di_status == 1 )
	{
		pref = text.substr(0,di);
		suff = text.substr(di+1);
	}
	else
	{
		pref = "";
		suff = text;
	}

	/*----------- SUFIX PROCESSING ----------------*/
	if( suff.size()==0 )
	{
		std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
		throw Exception_SyntaxError();
	}
	if( suff[0] == '^' and (suff.size()==1 or suff[1]!='^') )
	{
		if( suff.size()==1 )
		{
			std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
			throw Exception_SyntaxError();
		}
		_scope = SCOPE_UP;
		_name = name_t( suff.substr(1) );
	}
	else if( suff[0] == '_' and (suff.size()==1 or suff[1]!='_') )
	{
		if( suff.size()==1 )
		{
			std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
			throw Exception_SyntaxError();
		}
		_scope = SCOPE_DOWN;
		_name = name_t( suff.substr(1) );
	}
	else if( suff[0] == '=' and (suff.size()==1 or suff[1]!='=') )
	{
		if( suff.size()==1 )
		{
			std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
			throw Exception_SyntaxError();
		}
		_scope = SCOPE_SPOT;
		_name = name_t( suff.substr(2) );
	}
	else if( suff[0] == '^' and (suff.size()>1 and suff[1]=='^') )
	{
		if( suff.size()==2 )
		{
			std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
			throw Exception_SyntaxError();
		}
		_scope = SCOPE_PARENT;
		_name = name_t( suff.substr(2) );
	}
	else if( suff[0] == '_' and (suff.size()>1 and suff[1]=='_') )
	{
		if( suff.size()==2 )
		{
			std::cerr << "[e] event name is empty after parsing full name : "<<text<<" ["<<__FILE__<<":"<<__LINE__<<"] " << std::endl;
			throw Exception_SyntaxError();
		}
		_scope = SCOPE_CHILD;
		_name = name_t( suff.substr(2) );
	}
	else
	{
		_scope = SCOPE_GLOBAL;
		_name = name_t( suff );
	}

	/*----------- PREFIX PROCESSING ----------------*/

	string pref_reversed (pref);
	reverse(pref_reversed.begin(), pref_reversed.end());

	size_t dd = pref_reversed.find("/");

	if( dd == string::npos )
	{
		_context = context_t("");
		_channel = pref;
	}
	else
	{
		dd = pref.size()-dd;
		_context = context_t(pref.substr(0,dd));
		if( dd == pref.size()-1 )
		{
			_channel = "";
		}
		else
		{
			_channel = pref.substr(dd);
		}
	}

	if(_name.text.size()>0 and _name.text[0]!='/') _name.text = string("/")+_name.text;
	if(_context.text.size()>0 and _context.text[0]!='/') _context.text = string("/")+_context.text;

	/*------------- PARAMETERS -----------*/
	#ifdef VERSION_PARAMS_WITHOUT_SPLITTER
	vector<string> __name = _name.split();
	if(__name.size()>0 and __name.back().find('(')!=string::npos and __name.back().find(')')!=string::npos)
	{
		string back = __name.back();
		size_t iopen = back.find('(');
		size_t iclose = back.find(')');
		if( iopen<iclose)
		{
			//std::cout<<"[d] iopen = "<<iopen<<", iclose = "<<iclose<<std::endl;
			string params = back.substr(iopen+1, iclose-iopen-1);
			string real_back = back.substr(0,iopen);
			//std::cout<<"[d] params = "<<params<<", real_back = "<<real_back<<endl;
			__name.back() = real_back;
			_name = HierarchicalName( __name.begin(), __name.end() );
			//std::cout<<"[d] _name = "<<_name.full_name()<<endl;
			_parameters = split_params( params );
		}
	}
	#else
	if(_name.text.find('(')!=string::npos and _name.text.find(')')!=string::npos){
		string back = _name.text;
		size_t iopen = back.find('(');
		size_t iclose = back.find(')');
		if( iopen<iclose)
		{
			//std::cout<<"[d] iopen = "<<iopen<<", iclose = "<<iclose<<std::endl;
			string params = back.substr(iopen+1, iclose-iopen-1);
			string real_back = back.substr(0,iopen);
			//std::cout<<"[d] params = "<<params<<", real_back = "<<real_back<<endl;
			_name = HierarchicalName( real_back );
			//std::cout<<"[d] _name = "<<_name.full_name()<<endl;
			_parameters = params;//split_params( params );
		}
	}
	#endif

}

vector<string> Event::split_parameters(const string& path)const{
	vector<string> res;
	stringstream s;
	for(size_t i=0;i<(path).size();i++){
		if(path[i]==','){ res.push_back(to_lower_copy(s.str())); s.str(""); continue; }
		s << path[i];
	}
	res.push_back(to_lower_copy( trim_copy( s.str() )));
	return res;
}


bool Event::match(const Event& e)const
{
	if( is_template() and e.is_template() )
	{
		bool res = ( _name == e._name );
		//cout<<"[##] compare two templates("<<_name<<") and ("<<e._name<<"). res = "<<(res?"true":"false")<<endl;
		return res;
	}

	if( is_template() )
	{
		bool res = HierarchicalName(e.full_name()) == _name;
		//cout<<"[##] compare template("<<_name<<") with event ("<<e.full_name()<<"). res = "<<(res?"true":"false")<<endl;
		return res;
	}

	if( e.is_template() )
	{
		bool res = HierarchicalName(full_name()) == e._name;
		//cout<<"[##] compare template("<<e._name<<") with event ("<<full_name()<<"). res = "<<(res?"true":"false")<<endl;
		return res;
	}

	//cout<<"[##] compare events("<<e.full_name()<<") with event ("<<full_name()<<"). ";

	if( boost::to_lower_copy(_channel) != boost::to_lower_copy(e._channel) )
	{
		//cout<<"false: not same channels."<<endl;
		return false;
	}

	if( _name != e._name )
	{
		//cout<<"false: name is not same"<<endl;
		return false;
	}

	bool neigbors 			= abs( (long)_context.split().size() - (long)e._context.split().size() ) == 1;
	bool   s_up				=   _scope==SCOPE_UP    or   _scope==SCOPE_PARENT;
	bool   s_down 			=   _scope==SCOPE_DOWN  or   _scope==SCOPE_CHILD;
	bool e_s_up 			= e._scope==SCOPE_UP    or e._scope==SCOPE_PARENT;
	bool e_s_down 			= e._scope==SCOPE_DOWN  or e._scope==SCOPE_CHILD;
	bool   s_need_neigbor 	=   _scope==SCOPE_CHILD or   _scope==SCOPE_PARENT;
	bool e_s_need_neigbor 	= e._scope==SCOPE_CHILD or e._scope==SCOPE_PARENT;
	bool eq				 	= e._scope==SCOPE_SPOT  or   _scope==SCOPE_SPOT;

	if( (s_need_neigbor or e_s_need_neigbor) and not neigbors )
	{
		//cout<<"false: scope is not fitted."<<endl;
		return false;
	}

	bool _e 				= is_prefix(_context, e._context);
	bool e_ 				= is_prefix(e._context, _context);
	bool ee					= e._context.equals(_context);
//	bool ee					= e._context.equals( e._context );

	bool scope_ok =
		_scope==SCOPE_GLOBAL or e._scope==SCOPE_GLOBAL
		or
		(eq and ee)
		or
		(s_down and e_s_up and _e)
		or
		(s_up and e_s_down and e_)
		;
	if( scope_ok and (s_need_neigbor or e_s_need_neigbor) ) scope_ok = neigbors;

	if( not scope_ok )
	{
		//cout<<"false: scope is not fitted."<<endl;
		return false;
	}

	//cout<<"true"<<endl;
	return true;
}



}
}

std::ostream& operator<<(std::ostream& out, const cognitao::bus::Event& e){
	if( e.is_template() )
	{
		return out << "E[" << e.name().str() << "]";
	}
	return out<<e.full_name();
}



