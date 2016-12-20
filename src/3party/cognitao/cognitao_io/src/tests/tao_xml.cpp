
#include <iostream>
#include <sstream>
#include <fstream>
#include <cognitao/io/compiler/Compiler.h>
#include <cognitao/io/parser/xml/XMLParser.h>
#include <cognitao/io/compiler/fsm/FsmBuilder.h>
#include <cognitao/io/compiler/ftt/FttBuilder.h>
#include "Processor.h"

int main() {
	using namespace test;
	try{

		//PARSING
		cognitao::io::parser::xml::XMLParser parser;
		//cognitao::io::parser::core::MachinesCollection machines = parser.parse("/home/dan/workspace/cognitao/src/cognitao/cognitao_xml/xml/simple1.xml");
		//cognitao::io::parser::core::MachinesCollection machines = parser.parse("/home/dan/workspace/cognitao/src/cognitao/cognitao_xml/xml/fsm_one_state.xml");
		//cognitao::io::parser::core::MachinesCollection machines = parser.parse("/home/dan/workspace/cognitao/src/cognitao/cognitao_io/xml/A/startup.xml");
		cognitao::io::parser::core::MachinesCollection machines = parser.parse("/home/dan/workspace/cognitao/src/cognitao/cognitao_io/xml/vs.xml");

		std::cout<<"Start machine is "<< machines.start_machine << endl;
		BOOST_FOREACH(  cognitao::io::parser::core::MachinesCollection::Machines::value_type& m , machines.machines)
		{
			std::cout<<"-------------------"<<endl;
			print( cout, m.second );
		}


//		//COMPILATION
//		EventsStream estream;
//		Processor processor( estream );
//		cognitao::machine::Context context("test");
//
//		cognitao::io::compiler::Compiler compiler;
//
//		compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::fsm::FsmBuilder(processor) ) );
//		compiler.add_builder( cognitao::io::compiler::MachineBuilder::Ptr( new cognitao::io::compiler::ftt::FttBuilder(processor) ) );
//
//		cognitao::io::compiler::CompilationObjectsCollector collector;
//		cognitao::io::compiler::CompiledMachine ready_machine = compiler.compile( machines, collector );
//
//
//		//NOTE: PRINT OF CREATED MACHINE FOR DEBUG
//		cognitao::machine::graph::SimplePrinter printer;
//		printer.printers["fsm"] = boost::shared_ptr<SimplePrinterMachine>( new cognitao::machine::graph::SimplePrinterForFsm );
//		printer.printers["ftt"] = boost::shared_ptr<SimplePrinterMachine>( new cognitao::machine::graph::SimplePrinterForFtt );
//		printer.printers["parallel"] = boost::shared_ptr<SimplePrinterMachine>( new cognitao::machine::graph::SimplePrinterForParallel );
//		cognitao::machine::graph::Graph g;
//		set<void*> visited;
//		ready_machine->machine->state(g, context, visited);
//		printer.print( cout, g.search_root() ) << endl;

	}
	catch(const cognitao::io::parser::ParsingError& error)
	{
		std::cerr <<"ParsingError:"<<endl<< error.message <<endl;
	}
	catch(const cognitao::io::compiler::CompilerError& error)
	{
		std::cerr <<"CompilerError:"<<endl<< error.message <<endl;
	}

	return 0;
}
