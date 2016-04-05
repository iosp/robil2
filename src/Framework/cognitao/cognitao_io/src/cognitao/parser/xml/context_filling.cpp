
#include <cognitao/io/parser/xml/context_filling.h>
namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace context_filling{

using namespace core;

	void add_folder_to_path( Context& context, const string& folder )
	{
		//std::cout << "[i] add new folder to path "<<folder<<endl;
		context.path_list.push_back(folder);
	}

	void add_argument( Context& context, const string& arg_name, const string& arg_value )
	{
		//std::cout << "[i] add new argument "<<arg_name<<" : "<<arg_value<<endl;

		if(context.parameters.is_case_sensative)
			context.args_table[arg_name] = arg_value;
		else
			context.args_table[boost::to_lower_copy(arg_name)] = arg_value;
	}


}}}}}

