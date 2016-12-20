
#include <cognitao/io/parser/xml/io.h>

#include <cognitao/io/parser/xml/arguments_substitution.h>
#include <cognitao/io/parser/xml/context_filling.h>
#include <cognitao/io/parser/xml/sections.h>
#include <cognitao/io/parser/xml/subs.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace iostream{

using namespace sections;
using namespace tokens;

vector<string> split(const string& source, const string& del)
{
	vector<string> lines;
	split(lines, source, is_any_of(del));
	return lines;
}

std::string exec(const string cmd) {
	boost::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
	if (!pipe) return "ERROR";
	char buffer[128];
	std::string result = "";
	while (!feof(pipe.get())) {
		if (fgets(buffer, 128, pipe.get()) != NULL)
			result += buffer;
	}
	return result;
}


bool read_file( string file_name, string& xml )
{
	ifstream f( file_name.c_str() );
	if( f ){
		stringstream buffer;
		char c;
		while( true )
		{
			if( f.read(&c, 1) )
				buffer.write(&c, 1);
			else break;
		}
		xml = buffer.str();
		return true;
	}
	throw ParsingError()<<"Can not read file "<<file_name<<IN_FILE_LOCATION;
	return false;
}

bool parse_file( string file_name, property_tree::ptree& pt )
{
	Context context;
	MachinesTable machines;
	string start_machine;
	bool res = parse_file( file_name, context, machines, start_machine );
	std::cout<<"Result Document: \n  ------------------"<<endl;
	cout<<"  the structure is "<<endl<<to_string(machines[start_machine])<<endl;
	cout<<"  other machines are "<<endl;
	BOOST_FOREACH( const MachinesTable::value_type& m, machines )
	{
		cout<<"------: "<<m.first<<endl;
		cout<<to_string( m.second );
	}
	return res;
}




bool parse_file( string file_name, const Context& global_context, MachinesTable& machines, std::string& start_machine )
{
	boost::trim(file_name);
	if( file_name.empty() ) throw ParsingError()<<"File name is empty "<<file_name<<IN_FILE_LOCATION;

	string xml;
	PathList pathes;
	if( file_name[0]=='/' )
	{
		pathes.push_back("");
		file_name = file_name.substr(1);
	}
	else
	{
		pathes = global_context.path_list;
	}
	BOOST_FOREACH( string path, pathes )
	{
		try{
			if( path.empty()==false and path[path.size()-1]=='/' ) path = path.substr(0,path.size()-1);
			read_file( path + "/" + file_name , xml );
		}catch( const ParsingError& error )
		{
			continue;
		}
	}
	if( xml.empty() ) throw ParsingError()<<"Can not read file "<<file_name<<IN_FILE_LOCATION;

	stringstream stream(xml);

	return parse_file( stream, file_name, global_context, machines, start_machine );
}


bool parse_file( istream& stream, const string& source, const Context& global_context, MachinesTable& machines, std::string& start_machine )
{
	Context context = global_context;

	XMLTags input_tags;
	parse_to_tag_stream( stream, source , input_tags, global_context.parameters );
	TagPtr start = input_tags.begin();
	TagPtr end = input_tags.end();

	try{
			TagPair tao;
			if( find( "tao" , start, end, tao, Required, false ) )
			{
				tao_parsing( input_tags, tao , context );

				//machines.clear();
				machines.insert(context.machines_table.begin(), context.machines_table.end());
				machines.insert(global_context.machines_table.begin(), global_context.machines_table.end());
				start_machine = context.start_machine;

				return true;
			}

	}
	catch( const ParsingError& ex )
	{
		throw ex << "\n..."<<"tao parsing file "<<source<<" error " << IN_FILE_LOCATION;
	}

	return false;
}



bool parse_file_for_subs( string file_name, Context& global_context )
{
	boost::trim(file_name);
	if( file_name.empty() ) throw ParsingError()<<"File name is empty "<<file_name<<IN_FILE_LOCATION;

	string xml;
	PathList pathes;
	if( file_name[0]=='/' )
	{
		pathes.push_back("");
		file_name = file_name.substr(1);
	}
	else
	{
		pathes = global_context.path_list;
	}
	BOOST_FOREACH( string path, pathes )
	{
		try{
			if( path.empty()==false and path[path.size()-1]=='/' ) path = path.substr(0,path.size()-1);
			read_file( path + "/" + file_name , xml );
		}catch( const ParsingError& error )
		{
			continue;
		}
	}
	if( xml.empty() ) throw ParsingError()<<"Can not read file "<<file_name<<IN_FILE_LOCATION;

	stringstream stream(xml);

	return parse_file_for_subs( stream, file_name, global_context );
}

bool parse_file_for_subs( istream& stream, const string& source, Context& global_context )
{
	Context& context = global_context;

	XMLTags input_tags;
	parse_to_tag_stream( stream, source , input_tags, global_context.parameters );
	TagPtr start = input_tags.begin();
	TagPtr end = input_tags.end();

	try{
			TagPair tao;
			if( find( "tao" , start, end, tao, Required, false ) )
			{

				tao_parsing( input_tags, tao , context, PMP_DO_NOT_PARSE );

				return true;
			}

	}
	catch( const ParsingError& ex )
	{
		throw ex << "\n..."<<"tao parsing file "<<source<<" error " << IN_FILE_LOCATION;
	}

	return false;
}


bool include_file( string file_name, Context& global_context )
{
	boost::trim(file_name);
	if( file_name.empty() ) throw ParsingError()<<"File name is empty "<<file_name<<IN_FILE_LOCATION;

	string xml;
	PathList pathes;
	if( file_name[0]=='/' )
	{
		pathes.push_back("");
		file_name = file_name.substr(1);
	}
	else
	{
		pathes = global_context.path_list;
	}
	BOOST_FOREACH( string path, pathes )
	{
		try{
			if( path.empty()==false and path[path.size()-1]=='/' ) path = path.substr(0,path.size()-1);
			read_file( path + "/" + file_name , xml );
		}catch( const ParsingError& error )
		{
			continue;
		}
	}
	if( xml.empty() ) throw ParsingError()<<"Can not read file "<<file_name<<IN_FILE_LOCATION;

	stringstream stream(xml);

	return include_file( stream, file_name, global_context);
}


bool include_file( istream& stream, const string& source, Context& global_context )
{
	Context& context = global_context;

	XMLTags input_tags;
	parse_to_tag_stream( stream, source , input_tags, global_context.parameters );
	TagPtr start = input_tags.begin();
	TagPtr end = input_tags.end();

	try{
			TagPair tao;
			if( find( "tao" , start, end, tao, Required, false ) )
			{
				tao_parsing( input_tags, tao , context , PMP_OPTIONAL);

				return true;
			}

	}
	catch( const ParsingError& ex )
	{
		throw ex << "\n..."<<"tao parsing file "<<source<<" error " << IN_FILE_LOCATION;
	}

	return false;
}

}}}}}

