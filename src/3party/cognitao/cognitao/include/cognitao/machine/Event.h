/*
 * Events.h
 *
 *  Created on: Mar 4, 2015
 *      Author: dan
 */

#ifndef COGNITEAO_MACHINES_SRC_EVENT_H_
#define COGNITEAO_MACHINES_SRC_EVENT_H_

#include "core.h"
#include "Context.h"
#include "Printable.h"


namespace cognitao{
namespace machine{

struct Exception{};
struct Exception_UnknownExceptionType:public Exception{};
struct Exception_MachineIsNotInitilized:public Exception{};

class PrintableString{
public:
	std::string s;
	PrintableString(string s):s(s){}
	void print(ostream& out, const Context& context)const{out<<s;}
};

class PrintWithContext{
public:
	const Printable& p;
	const Context& c;
	PrintWithContext(const Printable& p, const Context& c):p(p),c(c){}
};

inline
ostream& operator<<(ostream& o, const PrintWithContext& e){
	e.p.print(o, e.c); return o;
}

template<class Printable>
ostream& operator<<(ostream& o, const boost::shared_ptr<Printable>& e){
	e->print(o); return o;
}



//class Events;

class Event{
public:
	enum Direction{ INCOME, OUTCOME };
	enum Scope{ GLOBAL, LOCAL, SUPER, PARENT, CHILD, SPOT };
	typedef string Channel;
	typedef string Name;
	typedef string Parameters;

protected:
	Direction _direction;
	Scope _scope;
	Channel _channel;
	Name _name;
	Parameters _parameters;
	Name _id;
	Context _context;

public:

	Event(const Event& e);
	Event(const Event& e, const Context& c);
	const Event& operator=(const Event& e);
	Event(const string& text);
	Event(const string& text, const Context& c);


	bool matches(const Event& e)const;
	bool matches_parameters(const Event& e)const;

private:
	static size_t dist( size_t a, size_t b )
	{
		if( a > b ) return a-b;
		return b-a;
	}
public:

	bool matches_context(const Event& e)const;

	const Direction& direction()const{ return _direction; }
	const Scope& scope()const{ return _scope; }
	const Channel& channel()const{ return _channel; }
	const Name& name()const{ return _name; }
	const Parameters& parameters()const{ return _parameters; }

	vector<string> split_parameters()const;

	const Context& context()const{ return _context; }
	void change_context(const Context& c){ _context = c; }

	void change_properties(Direction dir, Scope loc){
		_direction = dir;
		_scope = loc;
	}

	Event clone()const{
		Event e(*this);
		return e;
	}
	Event clone(Direction d)const{
		Event e(*this);
		e.change_properties(d, _scope);
		return e;
	}
	Event clone(Scope l)const{
		Event e(*this);
		e.change_properties(_direction, l);
		return e;
	}
	Event clone(Direction d, Scope l)const{
		Event e(*this);
		e.change_properties(d, l);
		return e;
	}

	void print(ostream& out)const;

	string str()const{
		stringstream s; print(s);
		return s.str();
	}

	class Exception{};
	class Exception_WrongName:public Exception{};

	void parse(const string& text);

	void parameters( const Parameters& param )
	{
		_parameters = param;
	}

};

inline
ostream& operator<<(ostream& out, const Event& e){
	e.print(out); return out;
}


}
}


#endif /* MACHINES_SRC_EVENTS_H_ */
