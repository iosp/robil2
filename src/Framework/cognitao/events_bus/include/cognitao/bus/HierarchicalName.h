/*
 * HierarchicalName.h
 *
 *  Created on: Nov 5, 2014
 *      Author: Dan Erusalimchik (danerde@gmail.com)
 */


#include <iostream>
#include <sstream>
#include <boost/regex.hpp>
#include <list>
#include <set>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/algorithm/string.hpp"
using namespace std;
using namespace boost;
using namespace boost::gregorian;
using namespace boost::posix_time;

#ifndef HierarchicalName_H_
#define HierarchicalName_H_


inline
posix_time::time_duration::sec_type epoch_time(boost::posix_time::ptime t)
{
    using namespace boost::posix_time;
    ptime epoch(boost::gregorian::date(1970,1,1));
    time_duration::sec_type x = (t - epoch).total_seconds();
    return x;
}

namespace cognitao {

class HierarchicalName{
public:
	std::string text;
	//HierarchicalName(const char* t):text(t){checkName();}
	HierarchicalName(const std::string& t = "/"):text(t){checkName();}
	HierarchicalName(const HierarchicalName& t):text(t.text){checkName();}
	HierarchicalName(const vector<string>& t);

	template<class CB>
	HierarchicalName(CB b, CB e)
	{
		text="";
		for( CB i=b; i!= e; i++)
		{
			text+=string("/")+(*i);
		}
		checkName();
	}

	bool is_template()const;
	void checkName();

	const HierarchicalName& operator=(const HierarchicalName& e){ text = e.text; return e;	}
	string to_regex()const;
	bool equals(const HierarchicalName& e)const;
	bool operator==(const HierarchicalName& e)const{ return equals(e); }
	bool operator!=(const HierarchicalName& e)const{ return !equals(e); }

	vector<string> split()const;

	int find(std::string m)const;

	HierarchicalName operator()( int start )const;
	HierarchicalName operator()( int start, int end )const;

	struct DIR{ bool is_head; size_t i; protected: DIR(bool h):is_head(h),i(0){} };
	struct Head:public DIR{ private:Head(size_t p):DIR(true){i=p;} public:Head():DIR(true){} Head operator+(size_t n)const{ return Head(n); } };
	struct Tail:public DIR{ private:Tail(size_t p):DIR(false){i=p;} public:Tail():DIR(false){} Tail operator-(size_t n)const{ return Tail(n); } };

	HierarchicalName operator()( const DIR& start, const DIR& end )const;

	HierarchicalName parent()const;
	std::string name()const;
	std::string full_name()const{ return text; }

	int size()const;
	HierarchicalName operator+(const HierarchicalName& e)const;
	HierarchicalName operator-(const HierarchicalName& e);
	bool operator<(const HierarchicalName& e)const;
	string substr(int s, int e)const;
	string substr(int s)const;
	string str()const{ return substr(0); }
	HierarchicalName removeSufix(std::string s)const;
	HierarchicalName removePrefix(std::string p)const;

	bool is_sufix_of( const HierarchicalName& o )const;
	bool is_prefix_of( const HierarchicalName& o )const;

	void to_lower();
	HierarchicalName to_lower_copy()const;

	bool is_root()const{ return text == "/"; }
	bool is_top()const{ return parent().is_root(); }
};

}

inline std::ostream& operator<<(std::ostream& out, const cognitao::HierarchicalName& e){
	return out<<e.str();
}
inline bool operator==(const std::string& s, const cognitao::HierarchicalName& e1){ return e1 == cognitao::HierarchicalName(s); }
inline bool operator!=(const std::string& s, const cognitao::HierarchicalName& e1){ return e1 == cognitao::HierarchicalName(s); }
inline bool operator==(const cognitao::HierarchicalName& e1, const std::string& s){ return e1 == cognitao::HierarchicalName(s); }
inline bool operator!=(const cognitao::HierarchicalName& e1, const std::string& s){ return e1 == cognitao::HierarchicalName(s); }


#endif /* HierarchicalName_H_ */
