/*
 * ParsingTools.h
 *
 *  Created on: May 6, 2014
 *      Author: dan
 */

#ifndef PARSINGTOOLS_H_
#define PARSINGTOOLS_H_

#include "PLPParser.h"

#include <iostream>
#include <sstream>
using namespace std;

static
Token combine(const vector<Token>& tokens, std::string delim=""){
	if(tokens.size()==0)return Token("",0,0,0);
	std::stringstream res;
	for(size_t i=0;i<tokens.size();i++){if(i>0)res<<delim; res<<tokens[i].name;}
	Token t = tokens.front(); t.name=res.str(); t.delimiter=false;
	return t;
}

static
void copy(InTokenStream& str, const vector<Token>& tokens){
	for(size_t i=0;i<tokens.size();i++)str<<tokens[i];
}

static
bool read_upto(vector<Token>& name_of, std::string stop, OutTokenStream& tokens, bool empty_error=true){
	Token t;
	while(tokens.eof()==false){
		tokens>>t;
		if(t.name==stop){
			if(name_of.empty()){
				if(empty_error){return false;}else{return true;}
			}else{
				return true;
			}
		}else{
			name_of.push_back(t);
		}
	}
	return false;
}

#define READ_ANY(t) if(tokens.eof()){tokens.set_error(__LINE__); return false;} else tokens>>t;
#define READ(t,NAME) READ_ANY(t) if(t.name!=NAME){tokens.set_error(__LINE__); return false;}
#define READ_KEY(t,NAME) { vector<Token> v; if(not read_upto(v,":",tokens)){tokens.set_error(__LINE__); return false;} t=combine(v," "); if(t.name!=NAME){tokens.set_error(__LINE__); return false;} }
#define READ_VALUE(t) { vector<Token> v; if(not read_upto(v," ",tokens,false)){tokens.set_error(__LINE__); return false;} t=combine(v," "); }

static
bool read_param(Token& t, OutTokenStream& tokens, string TYPE){
	READ_KEY(t,TYPE)  return true;
}

#define COLLECTION_READ_NAME(CNAME)\
	stage=0;\
	Token t;\
	READ_KEY(t, CNAME)\
	READ_ANY(t) if(not t.isNumber) return false;\
	READ(t, " ");\
	stage=1;
#define COLLECTION_ITEM_NAME(CNAME, CCOL, TYPE, CFIELD )\
		TYPE _item;\
		while(not tokens.eof()){\
			tokens.push();\
			READ_ANY(t);\
			if(t.name==" "){tokens.drop(); if(_item.CFIELD!=""){plp.CCOL.push_back(_item);_item=TYPE();} break;}else{tokens.pop();}\
			tokens.push(); read_param(t, tokens, CNAME);tokens.pop();\
			if(t.name==CNAME){ stage=2; if(_item.CFIELD!=""){plp.CCOL.push_back(_item);_item=TYPE();} }\
			COLLECTION_ITEM_REQ(CNAME, CFIELD)
#define COLLECTION_END\
		}\
	return true;

#define COLLECTION_ITEM_REQ(TYPE, F) tokens.push(); if(read_param(t, tokens, TYPE)){tokens.drop();READ_VALUE(t); set(_item,F,t.name);}else{tokens.pop();}
#define COLLECTION_ITEM_OPT(TYPE, F) tokens.push(); if(read_param(t, tokens, TYPE)){tokens.drop();READ_VALUE(t);  if(t.name!=""){set(_item,F,t.name);}else{ _item.F="N/A"; }}else{tokens.pop();}



#endif /* PARSINGTOOLS_H_ */
