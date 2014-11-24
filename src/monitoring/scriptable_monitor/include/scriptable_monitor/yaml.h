#ifndef YAML_INTERFACES_ADAPTER_H_
#define YAML_INTERFACES_ADAPTER_H_

#include <yaml-cpp/yaml.h>
#ifdef NODE_H_62B23520_7C8E_11DE_8A39_0800200C9A66
    #define YAML_OLD_INTERFACE
#else
    #define YAML_NEW_INTERFACE
#endif


#ifdef YAML_OLD_INTERFACE

    #define YAML_LOAD_DOCUMENT( NODE, FILE )\
        YAML::Parser parser(FILE);\
        parser.GetNextDocument(NODE);

    #define YAML_CLONE( NODE ) document.Clone()
    #define YAML_STREAM_TO( NODE,VAR ) (NODE) >> VAR
    
    #define YAML_AS( NODE, TYPE ) (NODE).to<TYPE>()

#else

    #define YAML_LOAD_DOCUMENT( NODE, FILE )\
        NODE = YAML::Load(FILE);
    #define YAML_CLONE( NODE ) std::auto_ptr<YAML::Node>( new YAML::Node(NODE) )
    
    template<class T> void __yaml_stream_to(const YAML::Node& n, T& t){ t=n.as<T>(); }
    #define YAML_STREAM_TO( NODE, VAR ) __yaml_stream_to((NODE),VAR)
    
    #define YAML_AS( NODE, TYPE ) (NODE).as<TYPE>()


#endif


#endif//YAML_INTERFACES_ADAPTER_H_