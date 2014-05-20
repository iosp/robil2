/*
 * Parameters.h
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <iostream>
#include <inttypes.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

typedef boost::shared_ptr<vector<string> > ParametersVectorPtr;


class YamlPtr {
public:
	YamlPtr(std::auto_ptr<YAML::Node> nodePtr) : _yamlNode(nodePtr) { }
	YamlPtr(const YamlPtr& yamlPtr) { this->_yamlNode = yamlPtr._yamlNode; }
	YamlPtr& operator=(const YamlPtr& yamlPtr) { this->_yamlNode = yamlPtr._yamlNode; return *this; }

	YAML::Node* get() { return _yamlNode.get(); }

	template <typename T> const T to() const { return _yamlNode->to<T>(); }

	template <typename T>
	inline const YAML::Node& operator [] (const T& key) const { return (*_yamlNode)[key]; }
	inline const YAML::Node& operator [] (const char *key) const { return (*_yamlNode)[key]; }
	inline const YAML::Node& operator [] (char *key) const { return (*_yamlNode)[key]; }
	size_t size() { return _yamlNode->size(); }

	string str() const {
		YAML::Emitter emitter;
		emitter << *_yamlNode;
		return emitter.c_str();
	}

private:
	mutable std::auto_ptr<YAML::Node> _yamlNode;
};

typedef YamlPtr Yaml;

class Parameters {
public:

	Parameters() {
		_parameters = boost::shared_ptr<vector<string> >(new vector<string>());
	}

	Parameters(vector<string> parameters) {
		_parameters = boost::shared_ptr<vector<string> >(new vector<string>(parameters));
	}

	Parameters(ParametersVectorPtr parameters) {
		_parameters = parameters;
	}

	size_t size() { return _parameters->size(); }

	template<typename T> T get(uint32_t index) { return T(); }
	template<typename T> void set(const T& value) { _parameters->push_back(boost::lexical_cast<string>(value)); }

private:
	ParametersVectorPtr _parameters;
};



#endif /* PARAMETERS_H_ */
