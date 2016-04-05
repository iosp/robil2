/*
 * Event.h
 *
 *  Created on: Nov 5, 2014
 *      Author: dan
 */

#ifndef EVENT_H_
#define EVENT_H_

#include "HierarchicalName.h"

namespace cognitao{
namespace bus{

class Event{
public:
	static const Event IDLE;

	enum Scope
	{
		SCOPE_GLOBAL, SCOPE_UP, SCOPE_DOWN, SCOPE_CHILD, SCOPE_PARENT, SCOPE_SPOT
	};

	typedef Scope 				scope_t;
	typedef HierarchicalName	name_t;
	typedef string 				channel_t;
	typedef HierarchicalName 	context_t;
	typedef string				source_t;
	typedef bool				internal_t;
	typedef posix_time::ptime	stamp_t;
	typedef std::string parameter_t;
	typedef std::vector<parameter_t>  parameters_t;

private:
	name_t		 _name;
	channel_t	 _channel;
	context_t	 _context;
	scope_t		 _scope;
	stamp_t 	 _stamp;
	source_t 	 _source;
	internal_t 	 _internal;
	parameter_t _parameters;

public:

	const name_t&  		name()const{ return _name; }
	const channel_t& 	channel()const{ return _channel; }
	const context_t& 	context()const{ return _context; }
	const scope_t& 		scope()const{ return _scope; }
	const stamp_t&  	stamp()const{ return _stamp; }
	const source_t&  	source()const{ return _source; }
	const internal_t& 	is_internal()const{ return _internal; }

	void stamp(const stamp_t& st){ _stamp = st; }
	void source(const source_t& src){ _source = src; }
	void select_as_internal(){ _internal = true; }
	void select_as_external(){ _internal = false; }

	Event();

	class Exception_SyntaxError: public std::exception{};
	class Exception_NameIsEmpty: public std::exception{};


	Event(
		  const name_t& 	name_
	);

	Event(
		  const string& 	name_
	);

	Event(
		  const name_t& 	  name_
		, const channel_t& 	  channel_
		, const context_t& 	  context_		= context_t("")
		, const scope_t& 	  scope_ 		= SCOPE_GLOBAL
		, const stamp_t& 	  stamp_ 		= stamp_t()
		, const source_t& 	  source_ 		= source_t("")
		, const internal_t&   internal_ 	= false
		, const parameter_t&  params_		= parameter_t()
	);

	Event(
		const Event& event
	);
	const Event& operator=(const Event& event);

	Event(
		  const Event& event
		, const channel_t& channel_
		, const context_t& context_
	);

	const parameter_t& parameters()const
	{
		return _parameters;
	}
	void parameters( const parameter_t& param )
	{
		_parameters = param;
	}

	string parameters_str()const;

	string full_name( bool with_params = true )const;

	void parse(const string& text);

	vector<string> split_parameters(const string& path)const;


    bool is_template()const{ return _name.is_template(); }

	private:
    static bool is_prefix(const context_t& p, const context_t& t)
    {
    	vector<string> pp = p.split();
    	vector<string> tt = t.split();

    	if( pp.size()> tt.size() ) return false;

    	for(size_t i=0;i<pp.size();i++)
    	{
    		if( boost::to_lower_copy(pp[i]) != boost::to_lower_copy(tt[i]) ) return false;
    	}

    	return true;
    }

	public:
    bool operator==(const Event& e)const{ return match(e); }

	public:
    bool match(const Event& e)const;

	bool operator!=(const Event& e)const
	{
		return not( *this == e );
	}


	bool operator==(const string& e)const{ return *this == Event(e); }
	bool operator!=(const string& e)const{ return *this != Event(e); }

	void stamp_to_now()
	{
		_stamp = posix_time::microsec_clock::local_time();
	}

	void stamp_to(ptime t)
	{
		_stamp = t;
	}

	long stamp_microsec()const
	{
		return epoch_time(_stamp);
	}

	double stamp_sec()const
	{
		return epoch_time(_stamp)/1000000.0;
	}

	void set_source(string address)
	{
		_source = address;
	}

	string get_source()const
	{
		return _source;
	}

	bool operator<(const Event& e)const
	{
		return full_name() < e.full_name();
	}

	Event internal_only()const
	{
		Event e(*this);
		e.select_as_internal();
		return e;
	}

	string to_regex()const
	{
		if(is_template()==false) return full_name();
		return _name.to_regex();
	}
};

}
}


std::ostream& operator<<(std::ostream& out, const cognitao::bus::Event& e);



#endif /* EVENT_H_ */
