/*
 * Events.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: dan
 */


#include <cognitao/machine/Events.h>

namespace cognitao{
namespace machine{

inline
void change_context( Event& event, const Context& context )
{
	if( event.context().empty() )
	{
		event.change_context( context );
		return;
	}
	Context ctx(event.context());
	Context::iterator i = std::find(ctx.begin(), ctx.end(), "~");
	if( i!=ctx.end())
	{
		size_t offset = i - ctx.begin();

		ctx.insert(ctx.begin()+(offset+1), context.begin(), context.end());

		ctx.erase(ctx.begin()+offset);
		event.change_context( ctx );
	}
}

Events Events::matches(const Event& event, const Context& _context)const{
	vector<Event> res;
	for(list<Event>::const_iterator i=_events.begin();i!=_events.end();i++){
		Event e(*i);
		change_context(e, _context);
		if( e.matches( event ) ){
			res.push_back(e);
		}
	}
	return Events(res);
}

Events Events::search(const Event::Direction& dir, const Context& _context)const{
	vector<Event> res;
	for(list<Event>::const_iterator i=_events.begin();i!=_events.end();i++){
		if(i->direction() != dir) continue;
		Event e(*i);
		change_context(e, _context);
		res.push_back(e);
	}
	return res;
}

void Events::print(ostream& out, const Context& context)const{
	if(_events.size()>1)out<<"{";
	string d="";
	for(list<Event>::const_iterator i=_events.begin();i!=_events.end();i++){
		Event e(*i);
		change_context(e, context);
		out<<d<<e;
		d=", ";
	}
	if(_events.size()>1)out<<"}";
}
void Events::print_multilines(ostream& out, const Context& context)const{
	if(_events.size()>1)out<<"{";
	string d="";
	for(list<Event>::const_iterator i=_events.begin();i!=_events.end();i++){
		Event e(*i);
		change_context(e, context);
		d=tab(context)+"\n";
		out<<e<<d;
	}
	if(_events.size()>1)out<<"}";
}

Events Events::replace_context( const Context& context )const
{
	Events res;
	BOOST_FOREACH( const Event& _e, _events )
	{
		Event e(_e);
		change_context(e, context);
		res.add( e );
	}
	return res;
}

}
}


