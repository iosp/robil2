
#include <cognitao/io/parser/xml/xml_args.h>
namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace xml_args{
using namespace core;

struct ArgToken{
	string type;
	string value;
};

typedef list<ArgToken> TokensList;

bool equals( const std::string& a, const std::string& b, bool is_case_sens )
{
	if( is_case_sens ) return a == b;
	return boost::to_lower_copy(a) == boost::to_lower_copy(b);
}
std::string copy( const std::string& a, bool is_case_sens )
{
	if( is_case_sens ) return a;
	return boost::to_lower_copy(a);
}

bool isChar( char c )
{
	if( c>='a' and c<='z' ) return true;
	if( c>='A' and c<='Z' ) return true;
	if( c>='0' and c<='9' ) return true;
	if( c=='+' or c=='-' or c=='.' ) return true;
	if( c=='_') return true;
	return false;
}
bool isSpace( char c )
{
	if( c==' ') return true;
	if( c=='\n') return true;
	if( c=='\r') return true;
	if( c=='\t') return true;
	return false;
}
bool isBS( char c )
{
	if( c=='\\') return true;
	return false;
}
bool isTagEnd( char c )
{
	if( c=='/') return true;
	return false;
}
bool isSetChar( char c )
{
	if( c=='=') return true;
	return false;
}

TokensList parse_tokens_of_args( const std::string& text )
{
	TokensList tokens;
	stringstream stream(text);
	stringstream value;
	int mode = 0;
	char c;
	ArgToken token;
	while( true )
	{
		stream.read(&c, 1);
		if( stream.eof() ) break;

		switch(mode)
		{
		case 0:
			if( isChar(c) )
			{
				mode = 1;
				value.write(&c, 1);
			}else
			if( isSpace(c) ){
			}else
			if( isTagEnd(c) ){
			}else
			{
				throw ParsingError() << "name of argument is not a char or space ["<<c<<":"<<int(c)<<"]" <<IN_FILE_LOCATION;
			}
			break;
		case 1:
			if( isChar(c) )
			{
				mode = 1;
				value.write(&c, 1);
			}else
			if( isSetChar(c) )
			{
				mode = 2;
				token.type = "key";
				token.value = value.str();
				//boost::to_lower(token.value);
				tokens.push_back( token );
				value.str("");
			}else
			if( isSpace(c) )
			{
				mode = 3;
				token.type = "key";
				token.value = value.str();
				//boost::to_lower(token.value);
				tokens.push_back( token );
				value.str("");
			}else
			{
				throw ParsingError() << "name of argument is not a char or space" <<IN_FILE_LOCATION;
			}
			break;
		case 2:
			if( isSpace(c) )
			{
				mode = 4;
			}else
			if( c == '"' )
			{
				mode = 5;
			}else
			if( c == '\'' )
			{
				mode = 6;
			}else
			if( isChar(c) )
			{
				mode = 7;
				value.write(&c, 1);
			}else
			{
				throw ParsingError() << "value argument char error" <<IN_FILE_LOCATION;
			}
			break;
		case 3:
			if( isSetChar(c) )
			{
				mode = 2;
			}else
			if( isSpace(c) ){
			}else
			{
				throw ParsingError() << "value argument char error" <<IN_FILE_LOCATION;
			}
			break;
		case 4:
			if( c == '"' )
			{
				mode = 5;
			}else
			if( c == '\'' )
			{
				mode = 6;
			}else
			if( isChar(c) )
			{
				mode = 7;
				value.write(&c, 1);
			}else
			if( isSpace(c) ){
			}else
			{
				throw ParsingError() << "value argument char error" <<IN_FILE_LOCATION;
			}
			break;
		case 5:
			if( c == '"' )
			{
				mode = 0;
				token.type = "value";
				token.value = value.str();
				value.str("");
				tokens.push_back( token );
			}else
			if( isBS(c) )
			{
				stream.read(&c,1);
				value.write(&c, 1);
			}else
			{
				value.write(&c, 1);
			}
			break;
		case 6:
			if( c == '\'' )
			{
				mode = 0;
				token.type = "value";
				token.value = value.str();
				value.str("");
				tokens.push_back( token );
			}else
			if( isBS(c) )
			{
				stream.read(&c,1);
				value.write(&c, 1);
			}else
			{
				value.write(&c, 1);
			}
			break;
		case 7:
			if( isSpace(c) )
			{
				mode = 0;
				token.type = "value";
				token.value = value.str();
				value.str("");
				tokens.push_back( token );
			}else
			if( isBS(c) )
			{
				stream.read(&c,1);
				value.write(&c, 1);
			}else
			{
				value.write(&c, 1);
			}
		}
	}
	if( mode == 7 )
	{
		mode = 0;
		token.type = "value";
		token.value = value.str();
		value.str("");
		tokens.push_back( token );
	}
	if( mode != 0 )
	{
		throw ParsingError() << "argument is not closed" << IN_FILE_LOCATION;
	}
	return tokens;
}

string get_xml_argument( TagPtr tag, const string& name, const Context& context )
{
	if( tag->name == Tag::DATA_NAME() ) throw ParsingError()<<"Can not find argument "<<name<<" in tag "<<tag->str();

	TokensList tokens;
	try{
		tokens = parse_tokens_of_args( tag->data );
	}catch( ParsingError& e )
	{
		throw e <<"\n..." <<"Syntax error in arguments of tag "<<tag->str()<<IN_FILE_LOCATION;
	}

	TokensList::const_iterator i1=tokens.begin();
	TokensList::const_iterator i2=i1; i2++;
	while(i2!=tokens.end())
	{
		if( i1->type=="key" and i2->type=="value" )
		{
			if( equals( i1->value, name, context.parameters.is_case_sensative) ) return i2->value;
		}
		else
		{
			throw ParsingError() <<"Syntax error in arguments of tag "<<tag->str()<<IN_FILE_LOCATION;
		}
		i1++; i2++; if(i2==tokens.end()) break;
		i1++; i2++; if(i2==tokens.end()) break;
	}

	throw ParsingError() <<"Argument "<<name<<" is not a part of arguments of tag "<<tag->str();

	return "";
}

bool contains_xml_argument( TagPtr tag, const string& name, const Context& context )
{
	if( tag->name == Tag::DATA_NAME() ) return false;

	TokensList tokens;
	try{
		tokens = parse_tokens_of_args( tag->data );
	}catch( ParsingError& e )
	{
		return false;
	}

	TokensList::const_iterator i1=tokens.begin();
	TokensList::const_iterator i2=i1; i2++;
	while(i2!=tokens.end())
	{
		if( i1->type=="key" and i2->type=="value" )
		{
			if( equals( i1->value, name, context.parameters.is_case_sensative) ) return true;
		}
		else
		{
			return false;
		}
		i1++; i2++; if(i2==tokens.end()) break;
		i1++; i2++; if(i2==tokens.end()) break;
	}

	return false;
}

string get_xml_argument( TagPtr tag, const string& name , string default_value, const Context& context )
{
	string ret = default_value;
	try{
		ret = get_xml_argument( tag, name, context);
	}catch(const ParsingError& e){}
	return ret;
}

map<string,string> get_xml_arguments( TagPtr tag, Requerments requered, const Context& context )
{
	map<string,string> result;

	if( tag->name == Tag::DATA_NAME() )
	{
		if( requered ) throw ParsingError()<<"Can not find arguments in tag "<<tag->str();
		else return result;
	}

	TokensList tokens;
	try{
		tokens = parse_tokens_of_args( tag->data );
	}catch( ParsingError& e )
	{
		if( requered ) throw e <<"\n..."<<"Syntax error in arguments of tag "<<tag->str()<<IN_FILE_LOCATION;
		return result;
	}

	TokensList::const_iterator i1=tokens.begin();
	TokensList::const_iterator i2=i1; i2++;
	while(i2!=tokens.end())
	{
		if( i1->type=="key" and i2->type=="value" )
		{
			result[ copy(i1->value, context.parameters.is_case_sensative) ] = i2->value;
		}
		else
		{
			if( requered ) throw ParsingError() <<"Syntax error in arguments of tag "<<tag->str()<<IN_FILE_LOCATION;
			return result;
		}
		i1++; i2++; if(i2==tokens.end()) break;
		i1++; i2++; if(i2==tokens.end()) break;
	}
	return result;
}

list<string> get_xml_arguments_names( TagPtr tag, Requerments requered, const Context& context )
{
	list<string> result;

	if( tag->name == Tag::DATA_NAME() )
	{
		if( requered ) throw ParsingError()<<"Can not find arguments in tag "<<tag->str();
		else return result;
	}

	TokensList tokens;
	try{
		tokens = parse_tokens_of_args( tag->data );
	}catch( ParsingError& e )
	{
		if( requered ) throw e <<"\n..." <<"Syntax error in arguments of tag "<<tag->str()<<IN_FILE_LOCATION;
		return result;
	}

	TokensList::const_iterator i1=tokens.begin();
	while(i1!=tokens.end())
	{
		if( i1->type=="key")
		{
			result.push_back( copy(i1->value, context.parameters.is_case_sensative)  );
		}
		i1++;
	}
	return result;
}

}}}}}
