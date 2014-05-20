/*
 * PLPParser.h
 *
 *  Created on: May 5, 2014
 *      Author: dan
 */

#ifndef PLPPARSER_H_
#define PLPPARSER_H_

#include <sstream>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <algorithm>
using namespace std;

class Token{
public:
	std::string name;
	int line,pose,index;
	bool delimiter;
	bool isNumber;
	Token():name(""),line(0),pose(0),index(0),delimiter(0),isNumber(0){}
	Token(std::string n, int l, int p, int i):name(n),line(l),pose(p),index(i),delimiter(true),isNumber(0){}
	void reset(int l, int p, int i, bool delim){name="";line=l;pose=p;index=i;delimiter=delim;}
};
inline
ostream& operator<<(ostream& o, const Token& t){
	return o<<"["<<t.name<<","<<t.index<<","<<t.line<<","<<t.pose<<","<<(t.delimiter?'d':(t.isNumber?'n':'w'))<<"]";
}


class OutTokenStream{
	vector<size_t> _stack;
public:
	OutTokenStream(const vector<Token>& _tokens):_tokens(_tokens),index(0), error_index(0), line(0){}
	const vector<Token>& _tokens;
	size_t index;
	size_t error_index;
	size_t line;

	OutTokenStream& operator>>(Token& t){ t = _tokens[index++]; return *this; }
	bool eof()const{ return _tokens.size()<=index; }
	void push(){_stack.push_back(index);}
	void pop(){index=_stack.back();_stack.pop_back();}
	void drop(){_stack.pop_back();}

	void set_error(int l=0){ error_index=index-1; line=l; }
	Token error(){ return _tokens[error_index]; }
};
class InTokenStream{
public:
	InTokenStream(vector<Token>& _tokens):_tokens(_tokens),index(0){}
	vector<Token>& _tokens;
	size_t index;
	InTokenStream& operator<<(const Token& t){ _tokens.push_back(t); return *this; }
	bool eof()const{ return _tokens.size()<=index; }
};

struct PLP;

class PLPParser {
public:
	PLPParser(std::istream& script_stream);
	virtual ~PLPParser();

	void read_all();
	bool create_structures();
	PLP& plp(){ return *_plp; }

private:
	void setPLP(PLP* p);

private:
	std::istream& _script_stream;
	vector<Token> _tokens;
	PLP* _plp;
};

#endif /* PLPPARSER_H_ */
