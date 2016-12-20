
#include <cognitao/io/parser/xml/sections.h>

#include <cognitao/io/parser/xml/args.h>
#include <cognitao/io/parser/xml/arguments_substitution.h>
#include <cognitao/io/parser/xml/context_filling.h>
#include <cognitao/io/parser/xml/io.h>
#include <cognitao/io/parser/xml/subs.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace sections{

static const std::string TAG_NAME_SECTION_MACHINES = "machines";
static const std::string TAG_NAME_SECTION_SUBS = "subs";
static const std::string TAG_NAME_SECTION_PATH = "path";
static const std::string TAG_NAME_SECTION_ARGS = "args";
static const std::string TAG_NAME_SECTION_INCLUDES = "includes";

static const std::string TAG_NAME_MACHINE = "machine";
static const std::string TAG_NAME_FOLDER = "folder";
static const std::string TAG_NAME_SUB = "sub";
static const std::string TAG_NAME_INCLUDE = "include";
static const std::string TAG_NAME_START_MACHINE = "root";

static const std::string PARAM_NAME_INCLUDE_FILE = "file";
static const std::string PARAM_NAME_ID = "id";

using namespace arguments;
using namespace context_filling;
using namespace tokens;
using namespace args;

void path_parsing( XMLTags& tags, TagPair path, Context& context )
{
	//NOTE: change from Optional to Required
	substitut_arguments(tags, path.open, path.closed, context, Optional );

	list<TagPtr> folders;
	if( find( TAG_NAME_FOLDER , path, folders, true ) )
	{
		BOOST_FOREACH( TagPtr f, folders )
		{
			TagPtr e;
			if( not search_closed_tag(f, path.closed, e) ) throw ParsingError()<<"Cann't close tag "<<f->str();

			if( f == e or (f+1) == e ) throw ParsingError()<<"data of tag "<<f->str()<<" is empty";

			string folder_data = to_string( f+1, e );
			boost::trim(folder_data);
			if( folder_data.empty() ) throw ParsingError()<<"data of tag "<<f->str()<<" is empty";

			add_folder_to_path( context, folder_data );
		}
	}
}

void includes_parsing( XMLTags& tags, TagPair includes, Context& context )
{
	//NOTE: change from Optional to Required
	substitut_arguments(tags, includes.open, includes.closed, context, Optional );

	list<TagPtr> includes_collection;
	if( find( TAG_NAME_INCLUDE , includes, includes_collection, true ) )
	{
		BOOST_FOREACH( TagPtr i, includes_collection )
		{
			TagPtr e;
			if( not search_closed_tag(i, includes.closed, e) ) throw ParsingError()<<"Cann't close tag "<<i->str();

			string include_file;

			if(include_file.empty())
			{
				XMLTags include_tags(i, e+1);
				if( xml_args::contains_xml_argument(include_tags.begin(), PARAM_NAME_INCLUDE_FILE, context) )
				{
					include_file = xml_args::get_xml_argument(include_tags.begin(), PARAM_NAME_INCLUDE_FILE, context);

				}
			}

			if( include_file.empty() and not ( i == e or (i+1) == e ) )
			{

				string folder_data = to_string( i+1, e );
				boost::trim(folder_data);
				include_file = folder_data;
			}


			if(include_file.empty() == false)
			{

				std::cout<<"[d] include external file : "<<include_file<<std::endl;

				Context& local_context = context;

				TagPair tag_of_sub;
				bool is_found = find( TAG_NAME_INCLUDE, i, e+1, tag_of_sub, Required, false );

				arg_parsing(tags, tag_of_sub, local_context, true);
				args_parsing(tags, tag_of_sub, local_context);

				if( iostream::include_file( include_file, local_context ) )
				{
					std::cout<<"[d] Success: loaded file"<<std::endl;
				}
				else
				{
					std::cout<<"[d] Failure: loaded file"<<std::endl;
				}

			}
		}
	}
}

void subs_parsing( XMLTags& tags, TagPair subs, Context& context )
{

	list<TagPtr> subs_collection;
	if( find( TAG_NAME_SUB , subs, subs_collection, true ) )
	{
		BOOST_FOREACH( TagPtr f, subs_collection )
		{
			TagPtr e;
			if( not search_closed_tag(f, subs.closed, e) ) throw ParsingError()<<"Cann't close tag "<<f->str();

			if( xml_args::contains_xml_argument(f, PARAM_NAME_INCLUDE_FILE, context) )
			{
				substitut_arguments(tags, f, e+1, context, Optional );
				string sub_file = xml_args::get_xml_argument(f, PARAM_NAME_INCLUDE_FILE, context);

				std::cout<<"[d] include subs from external file : "<<sub_file<<std::endl;

				Context& local_context = context;

				TagPair tag_of_sub;
				bool is_found = find( TAG_NAME_SUB, f, e+1, tag_of_sub, Required, false );

				arg_parsing(tags, tag_of_sub, local_context, true);
				args_parsing(tags, tag_of_sub, local_context);

				if( iostream::parse_file_for_subs( sub_file, local_context ) )
				{
					std::cout<<"[d] Success: loaded file with external subs"<<std::endl;
				}
				else
				{
					std::cout<<"[d] Failure: loaded file with external subs"<<std::endl;
				}
			}
			else
			{
				XMLTags sub_tags(f, e+1);
				string sub_id = xml_args::get_xml_argument(sub_tags.begin(), PARAM_NAME_ID, context);
				context.subs_table[sub_id] = sub_tags;
			}
		}
	}
}

void create_reference_tag( const string& id, const TagPair& target, Tag& tag, const Context& context )
{
	//string id = xml_args::get_xml_argument(target.open, PARAM_NAME_ID, context);
	tag.name = target.open->name;
	tag.data = PARAM_NAME_ID+"=\""+id+"\" /";
	tag.open_line = target.open->open_line;
	tag.open_offset_in_file = target.open->open_offset_in_file;
	tag.open_offset_in_line = target.open->open_offset_in_line;
	tag.close_line = target.closed->close_line;
	tag.close_offset_in_file = target.closed->close_offset_in_file;
	tag.close_offset_in_line = target.closed->close_offset_in_line;
	tag.source = target.open->source;
}

void move_machines_to_context( XMLTags& tags, TagPair machine, Context& context, bool search_inside = true)
{
	TagPtr cursor;
	TagPtr end;
	if(search_inside)
	{
		cursor = machine.open+1;
		end = machine.closed;
	}
	else
	{
		cursor = machine.open;
		end = machine.closed+1;
	}

	for(; cursor != end; cursor++)
	{
		if( cursor->name != TAG_NAME_MACHINE ) continue;
		TagPair sub_machine;
		bool find_tag = find( TAG_NAME_MACHINE, cursor, end, sub_machine, Required, false );
		if( find_tag )
		{
			bool include_ref = false;
			string file_name = "";

			try{
				file_name = xml_args::get_xml_argument(sub_machine.open, PARAM_NAME_INCLUDE_FILE, context);
				include_ref = true;
			}catch (const ParsingError& e) {
				include_ref = false;
			}

			if( include_ref )
			{
				Context local_context = context;
				arg_parsing(tags, sub_machine, local_context, true);
				args_parsing(tags, sub_machine, local_context);

				MachinesTable loaded_machines;
				std::string loaded_start_machine;
				if( iostream::parse_file( file_name, local_context, loaded_machines, loaded_start_machine ) )
				{
					BOOST_FOREACH( MachinesTable::value_type& _machine, loaded_machines)
					{
						string id = _machine.first;
						if(
								context.machines_table.find(id)!= context.machines_table.end()
								and
								context.machines_table[id].begin()->data != _machine.second.begin()->data
						)
						{
							cout<<"Warning "<<"included machine name("<<id<<") is already defined. "<<endl;
							cout<<"        "<<"include tag    is "   <<sub_machine.open->str()<<endl;
							cout<<"        "<<"current tag    is "   <<context.machines_table[id].begin()->str()<<endl;
							cout<<"        "<<"conflicted tag is "   <<_machine.second.begin()->str()<<endl;
						}
						XMLTags& mt = context.machines_table[id];
						mt.clear();
						mt.insert(mt.end(), loaded_machines[id].begin(), loaded_machines[id].end());
					}

					string id = loaded_start_machine;
					Tag tag;
					create_reference_tag( id, TagPair(loaded_machines[id].begin(), loaded_machines[id].end()-1), tag, context );

					tags.erase( sub_machine.open+1, sub_machine.closed+1 );
					*(sub_machine.open) = tag;
				}
			}

			if( not include_ref and sub_machine.has_body() )
			{
				cursor = sub_machine.closed+1;
				string id = xml_args::get_xml_argument(sub_machine.open, PARAM_NAME_ID, context);

				move_machines_to_context( tags, sub_machine, context );

				XMLTags& mt = context.machines_table[id];
				mt.clear();
				mt.insert(mt.end(), sub_machine.open, sub_machine.closed+1);

				Tag tag;
				create_reference_tag( id, sub_machine, tag, context );

				tags.erase( sub_machine.open+1, sub_machine.closed+1 );
				*(sub_machine.open) = tag;
			}else{
				string id = xml_args::get_xml_argument(sub_machine.open, PARAM_NAME_ID, context);
			}
		}
	}
}

bool is_machine_include_reference_load_it( XMLTags& tags, TagPair machine, Context& context )
{
	bool include_ref = false;
	string file_name = "";

	try{
		file_name = xml_args::get_xml_argument(machine.open, PARAM_NAME_INCLUDE_FILE, context);
		include_ref = true;
	}catch (const ParsingError& e) {
		include_ref = false;
	}

	if( include_ref )
	{
		move_machines_to_context( tags, machine, context, false );
		return true;
	}
	return false;
}

void machine_parsing( XMLTags& tags, TagPair machine, Context& context )
{
	if( is_machine_include_reference_load_it(tags, machine, context) )
	{
		return;
	}

	substitut_arguments( tags, machine.open+1, machine.closed, context, Required );

	apply_if_conditions( tags, machine, context );
	expand_blocks(tags, machine, context);
	subs::expand_subs(tags, machine, context);

	string tmp = xml_args::get_xml_argument(machine.open, PARAM_NAME_ID, context);

	move_machines_to_context( tags, machine, context );

	string machine_id = xml_args::get_xml_argument(machine.open, PARAM_NAME_ID, context);

	XMLTags& mt = context.machines_table[machine_id];
	mt.clear();
	mt.insert(mt.end(), machine.open, machine.closed+1);
}

void machines_parsing( XMLTags& tags, TagPair machines, Context& context )
{
	substitut_arguments( tags, machines.open, machines.closed, context, Required );

	list<TagPtr> machines_collection;
	if( find( TAG_NAME_MACHINE , machines, machines_collection, true ) )
	{
		BOOST_FOREACH( TagPtr f, machines_collection )
		{
			TagPtr e;
			if( not search_closed_tag(f, machines.closed, e) ) throw ParsingError()<<"Cann't close tag "<<f->str();

			machine_parsing( tags, TagPair(f, e), context );
		}
	} else {
		throw ParsingError() << "Found no machine tags in section "<<machines.open->str()<<IN_FILE_LOCATION;
	}
	TagPair root;
	if( find( TAG_NAME_START_MACHINE , machines, root, Required, true ) and root.has_body() )
	{
		context.start_machine = to_string(root.open+1, root.closed);
		if( context.machines_table.find(context.start_machine) == context.machines_table.end() )
		{
			throw ParsingError() << "root machine "<<context.start_machine<<" from tag "<<root.open->str()<<" does not defined."<<IN_FILE_LOCATION;
		}
	}
}

void tao_parsing( XMLTags& tags, TagPair tao, Context& context, ParsingMachinePolicy process_machines)
{
	TagPair found_tag;

	if( find( TAG_NAME_SECTION_PATH, tao, found_tag, Optional, true ) )
	{
		path_parsing( tags, found_tag, context);
	}

	if( find( TAG_NAME_SECTION_INCLUDES, tao, found_tag, Optional, true ) )
	{
		includes_parsing( tags, found_tag, context);
	}

	if( find( TAG_NAME_SECTION_ARGS, tao, found_tag, Optional, true ) )
	{
		Context local_context = context;
		args::args_parsing( tags, found_tag, local_context );
		//local args are just default values of arguments. if argument has been defined in context, don't replace its value.
		for( ArgsTable::const_iterator i=context.args_table.begin(); i!=context.args_table.end();i++)
			add_argument(local_context, i->first, i->second);
		context = local_context;
	}

	if( find( TAG_NAME_SECTION_SUBS, tao, found_tag, Optional, true ) )
	{
		subs_parsing( tags, found_tag, context );
	}

	switch(process_machines)
	{
	case PMP_PARSE:
		if( find( TAG_NAME_SECTION_MACHINES, tao, found_tag, Required, true ) )
		{
			machines_parsing( tags, found_tag, context );
		}
		break;
	case PMP_OPTIONAL:
		if( find( TAG_NAME_SECTION_MACHINES, tao, found_tag, Optional, true ) )
		{
			machines_parsing( tags, found_tag, context );
		}
		break;
	case PMP_DO_NOT_PARSE:
		break;
	}
}


}}}}}

