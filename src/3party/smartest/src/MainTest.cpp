#include <iostream>
#include <tr1/stdio.h>
#include <string>
#include <boost/filesystem.hpp>

#include "SFDP/SFDPobj.h"
#include "Generators/Gazebo/GazeboScenarioGenerator.h"
#include "Executor/GazeboExecutor.h"

#define PATH std::string("")



int main(int argc, char** argv)
{
			std::string SFV_root_file = PATH + argv[1];
			std::string WS_folder_path = PATH + argv[2];
			int SampNum = std::stoi(argv[3]);

			TiXmlElement * sampsXML = new TiXmlElement("samples");

			for (int sfv_i=1; sfv_i<=SampNum ; sfv_i++)
			{
				std::string scenario_folder_path = WS_folder_path +"/sfv_run_" + std::to_string(sfv_i);

				boost::filesystem::remove_all(scenario_folder_path);
				boost::filesystem::create_directory(scenario_folder_path);

				SFV * sfv = new SFV(SFV_root_file,scenario_folder_path);
				sfv->printToXML(scenario_folder_path+"/scen.SFV");
				sfv->execute(argc,argv);

				std::string sfv_name = "sfv_" + std::to_string(sfv_i);
				TiXmlElement * xml_sfv = new TiXmlElement( sfv_name );
				TiXmlElement * xml_grades = sfv->get_GradesAsXMLElement(0);
				xml_sfv->LinkEndChild(xml_grades);
				sampsXML->LinkEndChild(xml_sfv);
			}

			    std::string Grades_file_url = WS_folder_path + "/grades.xml";
				TiXmlDocument doc(Grades_file_url);
				TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
				doc.LinkEndChild(decl);
				doc.LinkEndChild(sampsXML);
				doc.SaveFile(Grades_file_url.c_str());
				std::cout << " printing Grades to file : " << Grades_file_url << std::endl;

			return 0;
}
