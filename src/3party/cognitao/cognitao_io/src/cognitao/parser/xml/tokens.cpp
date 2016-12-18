
#include <cognitao/io/parser/xml/tokens.h>
namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace tokens{

using namespace core;


	bool parse_to_tag_stream( istream& stream, std::string source, XMLTags& tags_stream, const ParsingParameters& params )
	{
		Tag tag;
		tag.source = source;
		stringstream buffer;
		char c;
		enum State{ Nothing, Name, Data, Args, Comment };
		State state = Nothing;

		size_t line=0;
		size_t line_offset=0;
		size_t file_offset=0;
		while(true)
		{
			if( stream.read(&c, 1) )
			{
				file_offset++;

				if( c=='\t' ) 		line_offset += params.tab_lenght;
				else if( c=='\r' ) 	line_offset	 = 0;
				else 				line_offset += 1;

				if(c=='\n')
				{
					line_offset=0;
					line++;
				}

				switch( state )
				{
				case Nothing:
					if( c=='<' )
					{
						tag.name = "";
						tag.open_line = line+1;
						tag.open_offset_in_line = line_offset;
						tag.open_offset_in_file = file_offset;
						state = Name;
						continue;
					}
					else
					{
						buffer.write(&c, 1);
						tag.name = Tag::DATA_NAME();
						tag.open_line = line+1;
						tag.open_offset_in_line = line_offset;
						tag.open_offset_in_file = file_offset;
						state = Data;
					}
					break;
				case Name:
					if( c!=' ' and c!='>' )
					{
						buffer.write(&c, 1);
						state = Name;
						if(buffer.str().size()==3 and buffer.str()=="!--")
						{
							tag.name = "!--";
							buffer.str("");
							state = Comment;
						}
						continue;
					}
					else if( c=='>' )
					{
						tag.name = buffer.str();
						tag.data="";
						tag.close_line = line;
						tag.close_offset_in_line = line_offset-1;
						tag.close_offset_in_file = file_offset-1;
						buffer.str("");
						boost::to_lower(tag.name);
						tags_stream.push_back(tag);
						state = Nothing;
					}
					else
					{
						tag.name = buffer.str();
						buffer.str("");
						state = Args;
					}
					break;
				case Args:
					if( c!='>' )
					{
						buffer.write(&c, 1);
						state = Args;
						continue;
					}
					else
					{
						tag.data = buffer.str();
						buffer.str("");
						tag.close_line = line;
						tag.close_offset_in_line = line_offset-1;
						tag.close_offset_in_file = file_offset-1;
						boost::to_lower(tag.name);
						tags_stream.push_back(tag);
						state = Nothing;
					}
					break;
				case Data:
					if( c=='<' )
					{
						tag.data = buffer.str();
						buffer.str("");
						tag.close_line = line;
						tag.close_offset_in_line = line_offset-1;
						tag.close_offset_in_file = file_offset-1;
						boost::to_lower(tag.name);
						tags_stream.push_back(tag);

						tag.name = "";
						tag.open_line = line+1;
						tag.open_offset_in_line = line_offset;
						tag.open_offset_in_file = file_offset;
						state = Name;
						continue;
					}
					else
					{
						buffer.write(&c, 1);
						state = Data;
					}
					break;
				case Comment:
					if( c!='>' )
					{
						buffer.write(&c, 1);
						state = Comment;
						continue;
					}
					else
					{
						if( boost::ends_with( buffer.str(), "--" ) ){
							tag.data = buffer.str();
							buffer.str("");
							tag.close_line = line;
							tag.close_offset_in_line = line_offset-1;
							tag.close_offset_in_file = file_offset-1;
							boost::to_lower(tag.name);
							tags_stream.push_back(tag);
							state = Nothing;
						}
						else
						{
							buffer.write(&c, 1);
							state = Comment;
						}
					}
					break;
				};
			}
			else break;
		}
		if(tag.name== Tag::DATA_NAME())
		{
			tag.data = buffer.str();
			tag.close_line = line;
			tag.close_offset_in_line = line_offset-1;
			tag.close_offset_in_file = file_offset-1;
			boost::to_lower(tag.name);
			tags_stream.push_back(tag);
		}

		return true;
	}

	bool is_normal_open_tag( const Tag& tag )
	{
		return tag.name.size()>0 and tag.name[0]!='?' and tag.name[0]!='!' and tag.name[0]!='#' and tag.name[0]!='/';
	}
	bool is_normal_closed_tag( const Tag& orig_tag, const Tag& tag )
	{
		return (tag.name == orig_tag.name and boost::ends_with(tag.data, "/")) or (tag.name == string("/")+orig_tag.name);
	}

	bool search_first_tag( TagPtr start, TagPtr end,  TagPtr& result )
	{
		result = start;
		while( result!=end and !is_normal_open_tag( *result ) )
		{
			result++;
		}
		return result!=end;
	}
	bool search_closed_tag( TagPtr start, TagPtr end,  TagPtr& result )
	{
		result = start;
		if( is_normal_closed_tag( *start, *result ) )
			return true;
		while( result!=end )
		{
	//		cout<<result->name<<" "<<(is_normal_open_tag(*result)?"open":"not open")<<" : "<<(is_normal_closed_tag( *start, *result )?"closed":"not closed")<<endl;
			if( is_normal_closed_tag( *start, *result ) and not is_normal_open_tag(*result) )
				break;
			if( start->name == result->name and start!=result and is_normal_open_tag(*result) ){
				bool res = search_closed_tag(result, end, result);
				if( not res )
					break;
			}
			result++;
		}
		return result!=end;
	}

	bool search_childs(  TagPtr start, TagPtr end, std::list<TagPtr>& results , bool search_sub_elements)
	{
		TagPtr end_tag;

		if( search_sub_elements  )
		{
			if( not is_normal_open_tag(*start) )
			{
				std::cout<<"[e] start element has to be normal open tag: "<<start->name<<" "<<start->open_line<<":"<<start->open_offset_in_line<<IN_FILE_LOCATION<<std::endl;
				return false;
			}
			if( not search_closed_tag(start, end, end_tag) )
			{
				std::cout<<"[e] cann't find closed tag for "<<start->name<<IN_FILE_LOCATION<<std::endl;
//				std::cout<<"--------{"<<endl;
//				std::cout<<to_string(start, end)<<endl;
//				std::cout<<"--------}"<<endl;
				return false;
			}
			if( start == end_tag ) return true;
			start++;
		}
		else
		{
			end_tag = end;
			if( start == end_tag ) return true;
		}

		while( start!=end_tag  and search_first_tag(start, end, start) ){
			if( end_tag != end and start->open_offset_in_file >= end_tag->open_offset_in_file ) break;
			results.push_back(start);
			TagPtr next;
			if(not search_closed_tag(start, end, next) )
			{
				ParsingError()<<"Cann't find closed tag for "<<start->str()<<IN_FILE_LOCATION;
				return false;
			}
			start = next;
			start++;
		}
		return true;
	}

	bool search_childs(  TagPair tag, std::list<TagPtr>& results , bool search_sub_elements)
	{
		return search_childs( tag.open, tag.closed+1, results, search_sub_elements);
	}

	bool find( string name, TagPtr start, TagPtr end, std::list<TagPtr>& results , bool search_sub_elements )
	{
		std::list<TagPtr> all;
		if( not search_childs(start, end, all, search_sub_elements) ) return false;
		int ic=0;
		BOOST_FOREACH( TagPtr i, all )
		{
			//cout<<"# "<<(ic++)<<". "<<i->name<<std::endl;
			if( i->name == name )
			{
				//cout<<"    add to results"<<endl;
				results.push_back(i);
			}
		}
		return results.size()>0;
	}
	bool find( string name, TagPtr start, TagPtr end, TagPtr& result , bool search_sub_elements )
	{
		std::list<TagPtr> results;
		if( not find( name, start, end, results, search_sub_elements ) ) return false;
		result = results.front();
		return true;
	}
	bool find( string name, TagPair tag, TagPtr& result , bool search_sub_elements )
	{
		return find( name, tag.open, tag.closed+1, result, search_sub_elements);
	}
	bool find( string name, TagPair tag, std::list<TagPtr>& results , bool search_sub_elements )
	{
		return find( name, tag.open, tag.closed+1, results, search_sub_elements);
	}

	bool find_all( string name, TagPtr start, TagPtr end, std::list<TagPtr>& results)
	{
		if( not find( name, start, end, results, false ) ) return false;

		std::list<TagPtr> all;
		if( not search_childs(start, end, all, true) ) return false;

		BOOST_FOREACH( TagPtr i, all )
		{
			if(not find_all( name, i, end, results ) ) return false;
		}
		return true;
	}
	bool find_all( string name, TagPtr start, TagPtr end, TagPtr& result )
	{
		std::list<TagPtr> results;
		if( not find_all( name, start, end, results ) ) return false;
		result = results.front();
		return true;
	}
	bool find_all( string name, TagPair tag, TagPtr& result )
	{
		return find_all( name, tag.open, tag.closed+1, result );
	}
	bool find_all( string name, TagPair tag, std::list<TagPtr>& results)
	{
		return find_all( name, tag.open, tag.closed+1, results );
	}

	string to_string( TagConstPtr start, TagConstPtr end )
	{
		stringstream s;
		for(; start!=end; start++)
		{
			s << start -> xml();
		}

		return s.str();
	}
	string to_string( TagPtr start, TagPtr end )
	{
		return to_string( (TagConstPtr)start, (TagConstPtr)end );
	}
	string to_string( const TagPair& p )
	{
		return to_string( p.open, p.closed+1 );
	}
	string to_string( const XMLTags& p )
	{
		return to_string( p.begin(), p.end() );
	}
	bool find( std::string name, TagPtr start, TagPtr end, TagPair& result, Requerments required, bool in_sub )
	{
		if( find( name, start, end, result.open, in_sub ) == false )
		{
			if(required==Required)
				throw ParsingError()<<"Cann't find "<<name<<" tag in "<<start->str()<<IN_FILE_LOCATION;
			cout<<"Warning "<<"Cann't find "<<name<<" tag in "<<start->str()<<endl;
			return false;
		}
		if( search_closed_tag( result.open, end, result.closed ) == false ) throw ParsingError()<<"Cann't close tag in "<<result.open->str();
		return true;
	}

	bool find( std::string name, TagPair tag, TagPair& result, Requerments required, bool in_sub )
	{
		return find( name, tag.open, tag.closed+1, result, required, in_sub );
	}


	void print( const Context& context )
	{
		cout<<"Path:"<<endl;
		BOOST_FOREACH( string f, context.path_list )
		{
			cout<<"   "<<f<<endl;
		}

		cout<<"Arguments:"<<endl;
		BOOST_FOREACH( ArgsTable::value_type f, context.args_table )
		{
			cout<<"   "<<f.first<<" = "<<f.second<<endl;
		}

		cout<<"Subs:"<<endl;
		BOOST_FOREACH( SubsTable::value_type f, context.subs_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}

		cout<<"Blocks:"<<endl;
		BOOST_FOREACH( BlocksTable::value_type f, context.blocks_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}

		cout<<"Machines:"<<endl;
		BOOST_FOREACH( MachinesTable::value_type f, context.machines_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}
		cout<<"   the start machine is "<<context.start_machine<<endl;
	}

	void print_path( const Context& context )
	{
		cout<<"Path:"<<endl;
		BOOST_FOREACH( string f, context.path_list )
		{
			cout<<"   "<<f<<endl;
		}
	}

	void print_args( const Context& context )
	{
		cout<<"Arguments:"<<endl;
		BOOST_FOREACH( ArgsTable::value_type f, context.args_table )
		{
			cout<<"   "<<f.first<<" = "<<f.second<<endl;
		}
	}
	void print_subs( const Context& context )
	{
		cout<<"Subs:"<<endl;
		BOOST_FOREACH( SubsTable::value_type f, context.subs_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}
	}
	void print_machines( const Context& context )
	{
		cout<<"Machines:"<<endl;
		BOOST_FOREACH( MachinesTable::value_type f, context.machines_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}
		cout<<"   the start machine is "<<context.start_machine<<endl;
	}

	void print_blocks( const Context& context )
	{
		cout<<"Blocks:"<<endl;
		BOOST_FOREACH( BlocksTable::value_type f, context.blocks_table )
		{
			cout<<"   "<<f.first<<"{\n"<< to_string(f.second.begin(), f.second.end()) <<"\n}"<<endl;
		}
	}

	void replace( XMLTags& tags, TagPtr start, TagPtr end, TagPtr new_start, TagPtr new_end )
	{
		tags.erase(start, end);
		tags.insert(end, new_start, new_end);
	}


}}}}}
