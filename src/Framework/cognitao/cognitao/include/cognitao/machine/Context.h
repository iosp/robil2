/*
 * Context.h
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_CONTEXT_H_
#define INCLUDE_COGNITAO_CONTEXT_H_

#include "core.h"
#include "Printable.h"

namespace cognitao{
namespace machine{

class Context{
protected:
	vector<string> _items;

public:
	Context(){}
	Context(const Context& c):_items(c._items){}
	Context(const string& path):_items(split(path)){}

	typedef vector<string>::iterator iterator;
	typedef vector<string>::const_iterator const_iterator;
	typedef vector<string>::value_type value_type;


	iterator begin(){ return _items.begin(); }
	iterator end(){ return _items.end(); }
	const_iterator begin()const{ return _items.begin(); }
	const_iterator end()const{ return _items.end(); }

	template<class CI>
	void insert( iterator i, CI s, CI e )
	{
		_items.insert(i, s, e);
	}
	void erase( vector<string>::iterator i)
	{
		_items.erase(i);
	}
	void clear(){ _items.clear(); }

	const Context& operator=(const Context& c){ _items=c._items; return c; }
	vector<string> split(const string& path)const{
		vector<string> res;
		stringstream s;
		foreachindex(i, path){
			if(path[i]=='/'){
				if(s.str()!="") res.push_back(to_lower_copy(s.str()));
				s.str("");
				continue;
			}
			s << path[i];
		}
		if(s.str()!="") res.push_back(to_lower_copy(s.str()));
		return res;
	}
	void push(const string& item){
		_items.push_back(to_lower_copy(item));
	}
	void pop(){
		_items.pop_back();
	}
	Context pop()const{
		Context c(*this);
		c.pop();
		return c;
	}
	Context operator+(const Context& c)const{
		Context cc(*this);
		for(size_t i=0;i<c._items.size();i++) cc.push(c._items[i]);
		return cc;
	}
	Context operator+(const string& c)const{
		Context cc(*this);cc.push(to_lower_copy(c));
		return cc;
	}
	void operator+=(const string& c){
		push(to_lower_copy(c));
	}
	string operator[](size_t i)const{ return _items[i]; }
	size_t size()const{ return _items.size(); }

	bool is_equals(const string& i1, const string& i2)const{
		if(i1=="*" or i2=="*") return true;
		return i1==i2;
	}
	bool is_prefix_of(const Context& c)const{
		if(size()>c.size()) return false;
		foreachindex(i, _items){
			if(not is_equals(_items[i], c._items[i])) return false;
		}
		return true;
	}
	bool is_suffix_of(const Context& c)const{
		if(size()>c.size()) return false;
		int o = c.size()-size();
		foreachindex(i, _items){
			if(not is_equals(_items[i], c._items[i+o])) return false;
		}
		return true;
	}
	bool is_equals(const Context& c)const{
		if(size()!=c.size()) return false;
		foreachindex(i, _items){
			if(not is_equals(_items[i], c._items[i])) return false;
		}
		return true;
	}
	const string& head()const{ return _items.front(); }
	const string& tail()const{ return _items.back(); }

	void print(ostream& out)const{
		const string d="/";
		foreachitem(const string & n, _items){
			out<<d<<n;
		}
	}
	bool empty()const{ return _items.empty(); }
	string str()const{
		stringstream s; print(s); return s.str();
	}
};

inline
string Printable::tab(const Context& n)const{
	stringstream nn; n.print(nn);
	stringstream s; while(s.str().size()<nn.str().size()) s<<" "; return s.str();
}

}
}

#endif /* INCLUDE_COGNITAO_CONTEXT_H_ */
