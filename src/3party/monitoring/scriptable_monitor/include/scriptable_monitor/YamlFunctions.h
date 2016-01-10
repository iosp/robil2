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
		 YAML::Node document;
		 YAML_LOAD_DOCUMENT(document, yaml_stream);
		 return YamlPtr(YAML_CLONE(document));
	}

};


#endif /* YAMLFUNCTIONS_H_ */
