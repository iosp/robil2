/*
 * HierarchicalName.cpp
 *
 *  Created on: Nov 5, 2014
 *      Author: Dan Erusalimchik (danerde@gmail.com)
 */


//#include <events_bus/events_bus.h>
#include <cognitao/bus/HierarchicalName.h>
#include <boost/algorithm/string.hpp>


#ifndef LICENSE_NOTIFICATION_DEF
#define LICENSE_NOTIFICATION_DEF
struct LICENSE_NOTIFICATION
{
	int print()
	{
		std::cout<<"**************************************"<<std::endl;
		std::cout<<"**************************************"<<std::endl;
		std::cout<<"**  Run-time licensing for BIU Labs **"<<std::endl;
		std::cout<<"**          by CogniTeam            **"<<std::endl;
		std::cout<<"**************************************"<<std::endl;
		std::cout<<"**************************************"<<std::endl;
		return 0;
	}
	LICENSE_NOTIFICATION()
	{
		static int i = print();
	}
};
static LICENSE_NOTIFICATION __license_notification;
#endif

namespace cognitao {


HierarchicalName::HierarchicalName(const std::vector<std::string>& t)
{
	text="";
	for(size_t i=0;i<t.size();i++)
	{
		text+=string("/")+t[i];
	}
	checkName();
}

bool HierarchicalName::is_template()const{
	return
			text.find('*')!=std::string::npos or
			text.find('?')!=std::string::npos or
			text.find('#')!=std::string::npos
	;
}
void HierarchicalName::checkName(){
	boost::trim(text);
	if(text.empty())
	{
		text = "/";
		return;
	}
	if(not is_template())
	{
		if(text[text.size()-1]=='/') text = text.substr(0,text.size()-1);
		if(text[0]!='/') text = string("/")+text;
	}
}


namespace __utils{
	string to_regex( std::string tmp)
	{
		const boost::regex r4("\\^"), r5("\\_"), r6("\\.");
		tmp = boost::regex_replace(tmp, r4, "\\\\^", boost::match_default|boost::format_sed);
		tmp = boost::regex_replace(tmp, r6, "\\\\.", boost::match_default|boost::format_sed);

		const boost::regex r1("\\*"), r2("\\?"), r3("\\#");
		tmp = boost::regex_replace(tmp, r1, ".*", boost::match_default|boost::format_sed);
		tmp = boost::regex_replace(tmp, r2, ".", boost::match_default|boost::format_sed);
		tmp = boost::regex_replace(tmp, r3, "[^/]*", boost::match_default|boost::format_sed);

		return std::string("^")+tmp+std::string("$");
	}
}

string HierarchicalName::to_regex()const
{
	return __utils::to_regex( boost::to_lower_copy(text) );
}
bool HierarchicalName::equals(const HierarchicalName& e)const{
	if(is_template() and e.is_template())
	{
		return boost::to_lower_copy(text)==boost::to_lower_copy(e.text);
	}
	if(not is_template() and not e.is_template())
	{
		return boost::to_lower_copy(text)==boost::to_lower_copy(e.text);
	}
	std::string tmp, txt;
	if(is_template()){
		tmp=boost::to_lower_copy(text); txt=boost::to_lower_copy(e.text);
	}
	else
	{
		tmp=boost::to_lower_copy(e.text); txt=boost::to_lower_copy(text);
	}

	tmp = __utils::to_regex(tmp);

	//cout<<"reg:"<<tmp<<endl;

	const boost::regex reg(tmp);
	return boost::regex_match(txt, reg);
}


vector<string> HierarchicalName::split()const{
	vector<string> res;
	stringstream s;
	for(int i=0;i<(int)text.size();i++){
		if(text[i]=='/'){
			if(s.str().empty()==false)res.push_back(s.str());
			s.str("");//s<<"/";
		}else{
			s<<text[i];
		}
	}
	res.push_back(s.str());
	return res;
}


int HierarchicalName::find(std::string m)const
{
	if(text == "/") return -1;
	HierarchicalName mm(m);
	if( mm.text == "/" ) return -1;
	vector<string> sp = split();
	for(size_t i=0; i<sp.size(); i++)
	{
		if( HierarchicalName(sp[i]) == mm ) return i;
	}
	return -1;
}
HierarchicalName HierarchicalName::operator()( int start )const
{
	vector<string> s = split();
	return HierarchicalName( s.begin() + start, s.end() );
}
HierarchicalName HierarchicalName::operator()( int start, int end )const
{
	vector<string> s = split();
	return HierarchicalName( s.begin() + start, s.begin() + end );
}
HierarchicalName HierarchicalName::operator()( const DIR& start, const DIR& end )const
{
	vector<string> s = split();
	size_t b = 0, e = s.size();
	if(start.is_head){ b = 0 + start.i; }else{ b = s.size()-start.i; }
	if(end.is_head){ e = 0 + end.i; }else{ e = s.size()-end.i; }
	return HierarchicalName( s.begin() + b, s.begin() + e );
}

std::string HierarchicalName::name()const{
	if(text=="/") return "";
	return (*this)(Tail()-1,Tail()).full_name().substr(1);
}
HierarchicalName HierarchicalName::parent()const{
	if(text=="/") return HierarchicalName();
	return (*this)(Head(),Tail()-1);
}

int HierarchicalName::size()const{ return text.size(); }
HierarchicalName HierarchicalName::operator+(const HierarchicalName& e)const
{
	std::string t1 = text;
	std::string t2 = e.text;
	if(is_template() == false )
	{
		if( t1 == "/" ) t1 = "";
	}
	return HierarchicalName( t1 + t2 );
}
HierarchicalName HierarchicalName::operator-(const HierarchicalName& e){
	regex r(e.text);
	std::string res = regex_replace(text,r,"",match_default|format_sed);
	return HierarchicalName(res);
}
bool HierarchicalName::operator<(const HierarchicalName& e)const{ return text<e.text; }
string HierarchicalName::substr(int s, int e)const{
	if(s<0) s = text.size()+s;
	if(e<0) s = text.size()+e;
	return text.substr(s, e-s);
}
string HierarchicalName::substr(int s)const{
	int e = (int)text.size();
	if(s<0) s = text.size()+s;
	if(e<0) s = text.size()+e;
	return text.substr(s, e-s);
}
HierarchicalName HierarchicalName::removeSufix(std::string s)const{
	size_t i = text.find(s);
	if(i==string::npos) return *this;
	size_t j=i;
	do{ i = text.find(s, i+1); if(i==string::npos) break; j=i; } while(true);
	return HierarchicalName(text.substr(0,j));
}
HierarchicalName HierarchicalName::removePrefix(std::string p)const{
	size_t i = text.find(p);
	if(i==string::npos) return *this;
	return HierarchicalName(text.substr(i+p.size()));
}

bool HierarchicalName::is_sufix_of( const HierarchicalName& o )const
{
	if( text.size() > o.text.size() ) return false;
	return text == o.text.substr(o.text.size()-text.size());
}

bool HierarchicalName::is_prefix_of( const HierarchicalName& o )const
{
	if(is_root()) return true;

	string t1 = text+"/";
	string t2 = o.text+"/";
	if( t1.size() > t2.size() ) return false;
	return t1 == t2.substr(0,t1.size());
}

void HierarchicalName::to_lower()
{
	boost::to_lower(text);
}
HierarchicalName HierarchicalName::to_lower_copy()const
{
	return HierarchicalName( boost::to_lower_copy(text) );
}


}  // namespace cognitao



