/*
 * YamlFunctions.h
 *
 *  Created on: Oct 29, 2013
 *      Author: blackpc
 */

#ifndef YAMLFUNCTIONS_H_
#define YAMLFUNCTIONS_H_

#include <scriptable_monitor/Parameters.h>


class YamlFunctions {
public:
	static Yaml toYaml(string object) {
		 stringstream yaml_stream;
		 yaml_stream << object;
		 YAML::Parser parser(yaml_stream);
		 YAML::Node document;
		 parser.GetNextDocument(document);
		 return YamlPtr(document.Clone());
	}

};


#endif /* YAMLFUNCTIONS_H_ */
