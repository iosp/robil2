

#include <cognitao/io/parser/xml/args.h>

#include <cognitao/io/parser/xml/arguments_substitution.h>
#include <cognitao/io/parser/xml/context_filling.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace args{
using namespace core;
using namespace arguments;
using namespace context_filling;
using namespace tokens;

void arg_parsing( XMLTags& tags,  TagPair arg_tag, Context& context, bool skip_id)
{
	string arg_name;
	string arg_value;

	if( arg_tag.open != arg_tag.closed and not skip_id)
	{
		string id = xml_args::get_xml_argument(arg_tag.open, "id", context);
		add_argument(context, id, to_string(arg_tag.open+1,arg_tag.closed));
	}

	map<string,string> vars = xml_args::get_xml_arguments(arg_tag.open, Required, context);
	for( map<string,string>::const_iterator i=vars.begin();i!=vars.end();i++ )
	{
		if( i->first != "id" ) add_argument( context, i->first, i->second);
	}
}

void args_parsing( XMLTags& tags, TagPair main_tag, Context& context )
{
	substitut_arguments(tags, main_tag.open, main_tag.closed, context, Required );

	list<TagPtr> args;
	if( find( "arg" , main_tag, args, true ) )
	{
		BOOST_FOREACH( TagPtr arg, args )
		{
			TagPtr e;
			if( not search_closed_tag(arg, main_tag.closed, e) ) throw ParsingError()<<"Cann't close tag "<<arg->str();
			arg_parsing( tags, TagPair(arg, e), context );
		}
	}

}

}}}}}




