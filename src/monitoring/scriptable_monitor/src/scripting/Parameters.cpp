/*
 * Parameters.cpp
 *
 *  Created on: Oct 28, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/Parameters.h>

template<> int 		Parameters::get<int>(uint32_t index) { return boost::lexical_cast<int>((*_parameters)[index]); }
template<> double 	Parameters::get<double>(uint32_t index) { return boost::lexical_cast<double>((*_parameters)[index]); }
template<> bool 	Parameters::get<bool>(uint32_t index) { return boost::lexical_cast<bool>((*_parameters)[index]); }
template<> string 	Parameters::get<string>(uint32_t index) { return (*_parameters)[index]; }
template<> YamlPtr 	Parameters::get<YamlPtr>(uint32_t index) {
	 stringstream yaml_stream;
	 yaml_stream << (*_parameters)[index];
	 YAML::Parser parser(yaml_stream);
	 YAML::Node document;
	 parser.GetNextDocument(document);
	 return YamlPtr(document.Clone());
}

template<> void 	Parameters::set<YAML::Node&>(YAML::Node& value) {
	YAML::Emitter emitter;
	emitter << value;
	_parameters->push_back(emitter.c_str());
}
