
#include <cognitao/io/parser/xml/arguments_substitution.h>

#include <cognitao/io/parser/xml/core.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>
#include <cognitao/io/parser/xml/ros.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace arguments{

using namespace core;
using namespace tokens;
using namespace ros;


	string substitut_argument( const string& text, const Context& context, Requerments req )
	{
		size_t sep = text.find(":");
		string type = "arg";
		string value = text;

		if( sep != string::npos)
		{
			type = text.substr(0,sep);
			value = text.substr(sep+1);
		}

		if( type == "arg"  )
		{
			if( context.parameters.is_case_sensative == false )
				boost::to_lower(value);

			if(context.args_table.find(value)!=context.args_table.end())
				return context.args_table.at(value);

			if(req == Required)
			{
				BOOST_FOREACH( const ArgsTable::value_type& v, context.args_table )
				{
					std::cout<<"# "<<v.first<<" = "<<v.second<<endl;
				}
				throw ParsingError()<<"Cann't find variable ["<<value<<"] in local context. \n..." <<IN_FILE_LOCATION;
			}
		}

		if( type == "rospack"  )
		{
			string path_to_package = search_ros_package( value );

			if(path_to_package.empty()==false)
				return path_to_package;

			if(req == Required)
			{
				throw ParsingError()<<"Cann't find ROS package ["<<value<<"]. \n..." <<IN_FILE_LOCATION;
			}
		}
		if( type == "rosparam"  )
		{
			string param_value = search_ros_parameter( value );

			return param_value;

			if(req == Required)
			{
				throw ParsingError()<<"Cann't find ROS parameter ["<<value<<"]. \n..." <<IN_FILE_LOCATION;
			}
		}

		if(req == Required) throw ParsingError()<<"Cann't substitute variable ["<<text<<"]";
		return "[[I_DO_NOT_KNOW_"+value+"_ARGUMENT]]";
	}

	void substitut_arguments( string& text , const Context& context, Requerments req )
	{
		size_t si = text.find("$");
		while( si!=string::npos )
		{
			size_t sn = text.find("{");
			if( sn == string::npos )
			{
				si = text.find("$", si+1);
				continue;
			}
			size_t ei = text.find("}",sn);
			if( ei == string::npos ) throw ParsingError()<<"Cann't close arguments substitution ";
			string result = text.substr(0,si) + substitut_argument(text.substr(sn+1, ei-sn-1), context, req) + text.substr(ei+1);
			text = result;
			si = text.find("$", ei);
		}
	}

	void substitut_arguments( XMLTags& tags, TagPtr start, TagPtr end, const Context& context, Requerments req )
	{
		for(; start!=end; start++)
		{
			if(start->name == Tag::COMENT_NAME()) continue;
			if(start->name == Tag::VERSION_NAME()) continue;

			try{
				substitut_arguments( start->data, context, req );

				if( start->name == Tag::DATA_NAME() )
				{
					std::stringstream s_data(start->data);
					XMLTags new_tags;
					if( parse_to_tag_stream(s_data, start->source, new_tags, context.parameters) )
					{
						size_t count = new_tags.size();
						if( (count>0 and new_tags.begin()->name!=Tag::DATA_NAME()) or count>1 )
						{
							tags.insert(start, new_tags.begin(), new_tags.end());
							TagPtr for_erase = start;
							start++;
							tags.erase(for_erase);
						}
					}
				}
			}catch( ParsingError& error )
			{
				throw error <<"\n... "<< "in tag "<<start->str();
			}
		}
	}


	void collect_blocks( XMLTags& tags, TagPair tag, Context& context )
	{
		TagPair b;
		for( b.open = tag.open; b.open!=tag.closed; b.open++ )
		{
			if( b.open->name == "block" )
			{
				if( search_closed_tag(b.open, tag.closed, b.closed ) )
				{
					if( b.has_body() ){
						string id = xml_args::get_xml_argument(b.open,"id", context);
						XMLTags& block = context.blocks_table[id];
						block.clear();
						block.insert(block.end(), b.open, b.closed+1);
						cout<<"Added "<<id<<": "<<to_string( b )<<endl;
						//b.open=b.closed;
					}
				}
			}
		}
	}
	void replace_blocks( XMLTags& tags, TagPair tag, Context& context )
	{
		list<TagPair> single_blocks;
		TagPair b;
		for( b.open = tag.open; b.open!=tag.closed; b.open++ )
		{
			if( b.open->name == "block" )
			{
				if( search_closed_tag(b.open, tag.closed, b.closed ) )
				{
					if( b.single() )
					{
						string id = xml_args::get_xml_argument(b.open, "id", context);
						if(context.blocks_table.find(id)!=context.blocks_table.end())
						{
							XMLTags& block = context.blocks_table[id];
							cout<<"Get block "<<id<<": "<<to_string(block)<<endl;
							replace( tags, b.open, b.closed+1, context.blocks_table[id].begin(), block.end() );
						}
						else
						{
							cout<<"Warning: try to get inexistent block. id "<<id<<" of "<<b.open->str()<<endl;
						}
					}
				}
			}
		}
	}
	void remove_blocks_envelope( XMLTags& tags, TagPair tag, Context& context )
	{
		list<TagPair> single_blocks;
		TagPair b;
		for( b.open = tag.open; b.open!=tag.closed; b.open++ )
		{
			if( b.open->name == "block" )
			{
				if( search_closed_tag(b.open, tag.closed, b.closed ) )
				{
					if( b.single() )
					{
						cout<<"Warning: some block does not replaced. "<<b.open->str()<<endl;
					}else{
						TagPtr t = b.open-1;
						tags.erase(b.open);
						tags.erase(b.closed);
						b.open=t;
					}
				}
			}
		}
	}

	void expand_blocks( XMLTags& tags, TagPair tag, const Context& global_context )
	{
		Context context = global_context;
		collect_blocks( tags, tag, context);
		replace_blocks( tags, tag, context);
		remove_blocks_envelope( tags, tag, context);
	}

	void apply_if_conditions( XMLTags& tags, TagPair tag, const Context& context )
	{
		TagPtr cursor = tag.open+1;
		TagPtr end = tag.closed;
		for(; cursor!=end; cursor++)
		{
			if(cursor->name!="if" and cursor->name!="unless") continue;
			//cout<<"IF OR UNLESS DETECTED: "<<cursor->name<<", "<<cursor->data<<endl;
			TagPair condition;
			if( find( cursor->name, cursor, end, condition, Required, false ) )
			{
				TagPtr next_to_open = condition.open+1;
				TagPtr next_to_closed = condition.closed+1;
				bool CNF = true;
				if( condition.has_body() )
				{
					typedef map<string, string> Args;
					Args args = xml_args::get_xml_arguments(condition.open, Required, context);
					BOOST_FOREACH( const Args::value_type& c, args)
					{
						print_args(context);
						string value = substitut_argument(c.first,context,Required);
						//cout<<"   TEST "<<value<<" == "<<c.second<<"   "<<((value == c.second)?"true":"false") << endl;
						CNF = CNF and (value == c.second);
						if( CNF == false ) break;
					}
					if( cursor->name == "unless" ) CNF = not( CNF );
				}
				else
				{
					CNF = false;
				}

				if( CNF )
				{
					std::cout<<"     remove only envelope"<<endl;
					cursor = next_to_open;
					tags.erase( condition.open );
					tags.erase( condition.closed );
				}
				else
				{
					std::cout<<"     remove full block"<<endl;
					cursor = next_to_closed;
					tags.erase( condition.open, next_to_closed );
				}
			}
		}
	}

}}}}}

