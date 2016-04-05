/*
 * Event.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: dan
 */

#include <cognitao/machine/Event.h>
#include <boost/regex.hpp>


namespace cognitao{
namespace machine{



Event::Event(const Event& e):
	_direction(e._direction),
	_scope(e._scope),
	_channel(e._channel),
	_name(e._name),
	_parameters(e._parameters),
	_id(e._id),
	_context(e._context)
{}
Event::Event(const Event& e, const Context& c):
	_direction(e._direction),
	_scope(e._scope),
	_channel(e._channel),
	_name(e._name),
	_parameters(e._parameters),
	_id(e._id),
	_context(c)
{}
const Event& Event::operator=(const Event& e){
	_direction=(e._direction),
	_scope=(e._scope),
	_channel=(e._channel),
	_name=(e._name),
	_parameters=(e._parameters),
	_id=(e._id),
	_context=(e._context);
	return e;
}
Event::Event(const string& text){
	parse(text);
}
Event::Event(const string& text, const Context& c)
{
	_context = c;
	parse(text);
}



bool Event::matches(const Event& e)const{
//	std::cout<<"[M] "<<(*this)<<" vs. "<<e
//			<<": d"<<_direction<<",ed"<<e._direction<<",c:"<<_channel<<",ec:"<<e._channel<<",n:"<<_name<<",en:"<<e._name
//			<<endl;
	if( _direction == e._direction ) return false;
	if( _channel != e._channel ) return false;
	if( _name != e._name ) return false;
	if( not matches_context(e) ) return false;
	if( not matches_parameters(e) ) return false;
	return true;
}
bool Event::matches_parameters(const Event& e)const{
	if( not _parameters.empty() and e._parameters.empty() ) return false;
	if( _parameters.empty() and not e._parameters.empty() ) return false;

	boost::sregex_iterator not_found;
	if(_parameters[0]=='@')
	{
		boost::regex expression1(_parameters);
		boost::sregex_iterator e1(e._parameters.begin(), e._parameters.end(), expression1);
		return e1 != not_found;
	}

	if(e._parameters[0]=='@')
	{
		boost::regex expression2(e._parameters);
		boost::sregex_iterator e2(  _parameters.begin(),   _parameters.end(), expression2);
		return e2 != not_found;
	}

	return (_parameters == e._parameters);
}

string t(bool e){ return e?"True":"Fasle"; }

bool Event::matches_context(const Event& e)const{
		bool _e =   _context.is_prefix_of(e._context);
		bool e_ = e._context.is_prefix_of(  _context);
		bool ee = e._context.is_equals   (  _context);
		bool neig = dist( _context.size() , e._context.size() ) == 1;
		bool req_neig = _scope == CHILD or _scope == PARENT;
		bool e_req_neig = e._scope == CHILD or e._scope == PARENT;

		bool waits_from_subs = _scope == LOCAL or _scope == CHILD;
		bool waits_from_suppers = _scope == SUPER or _scope == PARENT;
		bool waits_from_anyware = _scope == GLOBAL;

		bool e_for_subs = e._scope == LOCAL or e._scope == CHILD;
		bool e_for_suppers = e._scope == SUPER or e._scope == PARENT;
		bool e_for_anyware = e._scope == GLOBAL;


//		std::cout<<"[M] .. ctx : _e"<<_e<<",e_"<<e_<<",n"<<neig<<",rn"<<req_neig<<",ern"<<e_req_neig
//				<<",wsb"<<waits_from_subs
//				<<",wsp"<<waits_from_suppers
//				<<",wa" <<waits_from_anyware
//				<<",esb"<<e_for_subs
//				<<",esp"<<e_for_suppers
//				<<",ea" <<e_for_anyware
//		<<endl;
//		std::cout<<"[M] .... 1"<<endl;
		if( (req_neig or e_req_neig) and not neig ) return false;

		if( _scope==SPOT or e._scope==SPOT )
		{
			if( _scope==GLOBAL or e._scope==GLOBAL ) return true;
			if( _scope==SPOT and e._scope==SPOT and ee ) return true;
			return false;
		}

//		std::cout<<"[M] .... 2"<<endl;
		if(waits_from_subs and (e_for_suppers or e_for_anyware) and _e) return true;
//		std::cout<<"[M] .... 3"<<endl;
		if(waits_from_suppers and (e_for_subs or e_for_anyware) and e_) return true;
//		std::cout<<"[M] .... 4"<<endl;
		if(waits_from_anyware and e_for_suppers and _e) return true;
//		std::cout<<"[M] .... 5"<<endl;
		if(waits_from_anyware and e_for_subs    and e_) return true;
//		std::cout<<"[M] .... 6"<<endl;
		if(waits_from_anyware and e_for_anyware) return true;
//		std::cout<<"[M] .... 7"<<endl;
		return false;


//		if(  _scope == LOCAL and _e==false) return false;
//		if(e._scope == LOCAL and e_==false) return false;
//		if(  _scope == SUPER and e_==false) return false;
//		if(e._scope == SUPER and _e==false) return false;
//
//		if(  _scope == CHILD  and (_e==false or neig==false)) return false;
//		if(e._scope == CHILD  and (e_==false or neig==false)) return false;
//		if(  _scope == PARENT and (e_==false or neig==false)) return false;
//		if(e._scope == PARENT and (_e==false or neig==false)) return false;

		return true;
	}

vector<string> Event::split_parameters()const
{
	vector<string> res;
	stringstream s;
	foreachindex(i, _parameters){
		if(_parameters[i]==','){ res.push_back(to_lower_copy(s.str())); s.str(""); continue; }
		s << _parameters[i];
	}
	res.push_back(to_lower_copy(trim_copy( s.str() )));
	return res;

}

void Event::print(ostream& out)const{
	_context.print(out);
	out<<"/";
	out<<_channel;
	out<<(_direction==OUTCOME?'!':'?');
	out<<(_scope==SUPER ? "^" :
		  _scope==LOCAL ? "_" :
		  _scope==PARENT? "^^":
		  _scope==CHILD ? "__":
		  _scope==SPOT  ? "=" :
				  	  	  	  	"");
	out.flush();
	out<<_name;
	if(_parameters.size()>0){
		out<<"(";
//		int i=0;
//		foreachitem(const string& p, _parameters){
//			if(i++ > 0) out<<",";
//			out<<p;
//		}
		out<<_parameters;
		out<<")";
	}
	if(_id!=""){
		out<<"?"<<_id;
	}
}


//vector<string> Event::split_params(const string& path)const{
//	vector<string> res;
//	stringstream s;
//	foreachindex(i, path){
//		if(path[i]==','){ res.push_back(to_lower_copy(s.str())); s.str(""); continue; }
//		s << path[i];
//	}
//	res.push_back(to_lower_copy(trim_copy( s.str() )));
//	return res;
//}

void Event::parse(const string& text){
		size_t _lout = text.find('!'); bool _lout_b = _lout!=string::npos;
		size_t _lin = text.find('?'); bool _lin_b = _lin!=string::npos;
		if(_lout_b and _lin_b and _lin<_lout){
			cout<<"[w] wrong name of event. ? before !"<<"  IN["<<__FILE__<<":"<<__LINE__<<":"<<__FUNCTION__<<"]"<<endl;
			throw Exception_WrongName();
		}

		size_t n = _lout_b ? _lout+1 : _lin_b ? _lin+1 : 0;

		if(n == text.size()){
			cout<<"[w] wrong name of event. empty name"<<"  IN["<<__FILE__<<":"<<__LINE__<<":"<<__FUNCTION__<<"]"<<endl;
			throw Exception_WrongName();
		}
		bool _lsup_b = text[n] == '^';
		bool _lloc_b = text[n] == '_';
		bool _lspo_b = text[n] == '=';
		bool _lsupsup_b = _lsup_b and n+1<text.size() and text[n+1] == '^';
		bool _llocloc_b = _lloc_b and n+1<text.size() and text[n+1] == '_';

		size_t nn = n;
		if(_lsup_b or _lloc_b or _lspo_b) nn = n+1;
		if(_lsupsup_b or _llocloc_b) nn = nn+1;

		size_t _lpar = text.find('(',n+1); bool _lpar_b = _lpar!=string::npos;

		string reversed = text;
		std::reverse(reversed.begin(), reversed.end());
		size_t _lparc = reversed.find(')'); bool _lparc_b = _lparc!=string::npos;
		if(_lparc_b) _lparc = text.size()-1 - _lparc;

		if(_lpar_b){
			if(not _lparc or _lparc<_lpar or _lpar < n+1){
				cout<<"[w] wrong name of event. parameters syntax wrong"<<"  IN["<<__FILE__<<":"<<__LINE__<<":"<<__FUNCTION__<<"]"<<endl;
				throw Exception_WrongName();
			}
		}

		if(_lout_b) _direction = OUTCOME;
		else _direction = INCOME;

		if(_lsupsup_b) _scope = PARENT;
		else if(_llocloc_b) _scope = CHILD;
		else if(_lsup_b) _scope = SUPER;
		else if(_lloc_b) _scope = LOCAL;
		else if(_lspo_b) _scope = SPOT;
		else _scope = GLOBAL;

		if( _lin_b or _lout_b ){
			_channel = text.substr( 0, (_lout_b ? _lout : _lin) );
			trim(_channel);
			to_lower(_channel);
			if(_channel.empty() == false and _channel[0]=='/' )_channel = _channel.substr(1);
			if(_channel.empty() == false)
			{
				list<string> pel;
				boost::split(pel, _channel, is_any_of("/"));
				if( pel.size() > 1 )
				{
					_channel = pel.back();
					_context.insert(_context.end(), pel.begin(), pel.end() );
				}
			}
		}

		if( _lpar_b ){
			_name = text.substr(nn, _lpar-nn);
		}else if( _lin_b and _lout_b ){
			_name = text.substr(nn, _lin-nn);
		}else{
			_name = text.substr(nn);
		}
		if( _name.empty()==false and _name[0]=='/' ) _name = _name.substr(1);

		if(_lpar_b and _lin_b and _lout_b and _lpar > _lin){
			cout<<"[w] wrong name of event. ? before parameters"<<"  IN["<<__FILE__<<":"<<__LINE__<<":"<<__FUNCTION__<<"]"<<endl;
			throw Exception_WrongName();
		}
		if(_lpar_b and _lin_b and _lout_b and _lparc > _lin){
			_lin_b = false;
		}

		if(_lpar_b and _lin_b and _lout_b and _lpar < _lin){
			_id = text.substr(_lin+1);
		}
		if(not _lpar_b and _lin_b and _lout_b){
			_id = text.substr(_lin+1);
		}

		if(_lpar_b and _lparc_b){
			_parameters = text.substr(_lpar+1, _lparc-_lpar-1) ;
		}

		to_lower(_channel);
		to_lower(_name);
		to_lower(_id);
	}






}
}




