/*
 * PlpLoader.h
 *
 *  Created on: Jun 9, 2014
 *      Author: dan
 */

#ifndef PLPLOADER_H_
#define PLPLOADER_H_
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <list>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;

namespace plp{


class PlpLoader {
public:

	class Exception{
	public:
		virtual ~Exception(){}
	};
	class Exception_FileLoadProblem:public Exception{};
	class Exception_PlpHasNoName:public Exception{};
	class Exception_PlpIsNotRepeatable:public Exception{};

	PlpLoader(string filename);
	PlpLoader(istream& stream);
	PlpLoader();
	void load_plp(istream& stream);
	void load_plp(string filename);
	void parse();
	string plp_name()const;
	string plp_type()const;
	bool plp_is_repeated()const;
	string get_script()const{ stringstream s; s<<(is_need_header()?_header:"") << "\n" <<_plp_text; return s.str();}
	bool is_need_header()const;
	vector<string> search(string key)const{
		vector<string> res;
		for(map<string,string>::const_iterator i=_plp_map.begin();i!=_plp_map.end();i++){
			if(starts_with(i->first, key)) res.push_back(i->second);
		}
		return res;
	}
	string operator[](string key)const{
		if(_plp_map.find(key)==_plp_map.end()) return "";
		return _plp_map.at(key);
	}
	virtual ~PlpLoader();
	map<string,string>& plp_map(){ return _plp_map; }
	const map<string,string>& plp_map()const{ return _plp_map; }
private:
	string _plp_text;
	map<string,string> _plp_map;
	string _header;
};

}

#endif /* PLPLOADER_H_ */
