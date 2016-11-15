

#include <cognitao/io/parser/xml/subs.h>

#include <cognitao/io/parser/xml/args.h>
#include <cognitao/io/parser/xml/arguments_substitution.h>
#include <cognitao/io/parser/xml/context_filling.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace subs{
using namespace core;
using namespace arguments;
using namespace context_filling;
using namespace args;
using namespace tokens;

void sub_parsing( XMLTags& tags, TagPair sub, const Context& context )
{
	TagPair args;
	Context local_context = context;
	try{
		if( find( "args", sub, args, Optional, true ) )
		{
			args_parsing( tags, args, local_context );

			for( ArgsTable::const_iterator i=context.args_table.begin(); i!=context.args_table.end();i++)
			{
				add_argument(local_context, i->first, i->second);
			}
		}
		TagPair block;
		if( find( "block", sub, block, Required, true) )
		{
			if( block.has_body() )
			{
				substitut_arguments( tags, block.open+1, block.closed, local_context, Required );
				apply_if_conditions( tags, TagPair(block.open+1, block.closed-1), local_context);
				expand_blocks(tags, TagPair(block.open+1, block.closed-1), local_context);
			}
		}
	}catch(ParsingError& error)
	{
		throw error <<"\n..."<<"sub parsing error for tag "<<sub.open->str()<<IN_FILE_LOCATION;
	}
}

void sub_parsing( const std::string sub_id, const Context& context, XMLTags& tags)
{
	SubsTable::const_iterator i = context.subs_table.find(sub_id);
	if( i == context.subs_table.end() )
	{
		throw ParsingError()<<"Sub with id "<<sub_id<<" does not found."<<IN_FILE_LOCATION;
	}

	XMLTags template_tags( i->second.begin(), i->second.end() );
	sub_parsing( template_tags, TagPair( template_tags.begin(), template_tags.end()-1 ), context );
	tags = template_tags;

}

void expand_subs( XMLTags& tags, TagPair& main_tag, const Context& context)
{
	TagPtr cursor = main_tag.open;
	TagPtr end = main_tag.closed; end++;
	for(; cursor!=end; cursor++)
	{
		if( cursor->name != "sub" ) continue;
		TagPair subtag(cursor, cursor);
		if( search_closed_tag( cursor, end, subtag.closed ) ){

			TagPtr next_to_closed = subtag.closed+1;
			Context lcontext = context;

			string sub_id = xml_args::get_xml_argument(cursor, "id", context);
			arg_parsing(tags, subtag, lcontext, true);
			args_parsing(tags, subtag, lcontext);

			XMLTags sub_expanded;
			sub_parsing(sub_id,lcontext,sub_expanded);
			TagPair block;
			if( find("block", sub_expanded.begin(), sub_expanded.end(), block, Required, true) )
			{
				replace( tags, cursor, next_to_closed, block.open+1, block.closed );
			}

			cursor = next_to_closed;
		}

	}
}


}}}}}


