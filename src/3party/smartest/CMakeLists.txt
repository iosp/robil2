cmake_minimum_required(VERSION 2.8.3)
project(smartest)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  rosbag
  robil_msgs 
  gazebo_msgs
  fcl 
)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

find_package(TinyXML REQUIRED)

find_package(OpenCV REQUIRED)
                            
find_package(Gazebo REQUIRED)

FIND_PACKAGE(PythonLibs 2.7 REQUIRED)

catkin_package(
    #LIBRARIES  
    INCLUDE_DIRS
      include
    DEPENDS
      TinyXML
  CATKIN_DEPENDS      
      message_runtime 
      robil_msgs 
)

                                                                                                                                                 
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIR} /usr/include/sdformat-4.3/sdf )



############
## UTILS  ##
############

add_library(NumberSampler src/utils/NumberSampler.cpp)


##############
## RESOURCE ##
##############
add_library(ResourceHandler src/Resource/ResourceHandler.cpp)
target_link_libraries(ResourceHandler ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_library(TerrainAnalyzer src/Generators/Gazebo/TerrainAnalyzer.cpp)
target_link_libraries(TerrainAnalyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
#### RULES ##
#############
add_library(RULE_wp_path_inside_map src/Rules/RULE_wp_path_inside_map.cpp)
target_link_libraries(RULE_wp_path_inside_map ${catkin_LIBRARIES}
TerrainAnalyzer ResourceHandler
)

add_library(RULE_platform_init_pose_with_no_obj_colisions src/Rules/RULE_platform_init_pose_with_no_obj_colisions.cpp)
target_link_libraries(RULE_platform_init_pose_with_no_obj_colisions ${catkin_LIBRARIES}
TerrainAnalyzer ResourceHandler
)




#############
### SFDP ####
#############
add_library(ScenarioFeatureGroup src/SFDP/ScenarioFeatureGroup.cpp)
add_library(ScenarioFeature src/SFDP/ScenarioFeature.cpp)

target_link_libraries(ScenarioFeature ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
NumberSampler
)


target_link_libraries(ScenarioFeatureGroup  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
ScenarioFeature
)


add_library(SFDPobj src/SFDP/SFDPobj.cpp)
target_link_libraries(SFDPobj ${catkin_LIBRARIES} 
ScenarioFeatureGroup
ScenarioFeature
SFV
#GazeboExecutor
#GazeboScenarioGenerator
)


#############
#### SFV ####
#############

add_library(sfvSubGroup src/SFV/sfvSubGroup.cpp)
target_link_libraries(sfvSubGroup ${catkin_LIBRARIES}
)

add_library(SFVwp src/SFV/SFVwp.cpp)
target_link_libraries(SFVwp ${catkin_LIBRARIES}
sfvSubGroup
)

add_library(SFVpath src/SFV/SFVpath.cpp)
target_link_libraries(SFVpath ${catkin_LIBRARIES}
ScenarioFeature
sfvSubGroup
SFVwp
)

add_library(SFVmass_link src/SFV/SFVmass_link.cpp)
target_link_libraries(SFVmass_link ${catkin_LIBRARIES}
sfvSubGroup
)

add_library(SFVfriction_link src/SFV/SFVfriction_link.cpp)
target_link_libraries(SFVfriction_link ${catkin_LIBRARIES}
sfvSubGroup
)

add_library(SFVsensor_link src/SFV/SFVsensor_link.cpp)
target_link_libraries(SFVsensor_link ${catkin_LIBRARIES}
sfvSubGroup
)


add_library(SFVterraine src/SFV/SFVterraine.cpp)
target_link_libraries(SFVterraine ${catkin_LIBRARIES}
sfvSubGroup
)


add_library(SFVObject src/SFV/SFVObject.cpp)
target_link_libraries(SFVObject ${catkin_LIBRARIES}
sfvSubGroup
)

add_library(SFVobjScattering src/SFV/SFVobjScattering.cpp)
target_link_libraries(SFVobjScattering ${catkin_LIBRARIES}
sfvSubGroup
SFVObject
)

add_library(SFVobstacaleOnPath src/SFV/SFVobstacaleOnPath.cpp)
target_link_libraries(SFVobstacaleOnPath ${catkin_LIBRARIES}
sfvSubGroup
ScenarioFeatureGroup
)

add_library(SFVobsOnPathScattering src/SFV/SFVobsOnPathScattering.cpp)
target_link_libraries(SFVobsOnPathScattering ${catkin_LIBRARIES}
sfvSubGroup
SFVobstacaleOnPath
)


add_library(SFVplatformPose src/SFV/SFVplatformPose.cpp)
target_link_libraries(SFVplatformPose ${catkin_LIBRARIES}
sfvSubGroup
)


add_library(SFV src/SFV/SFV.cpp)
target_link_libraries(SFV ${catkin_LIBRARIES}
ScenarioFeatureGroup
sfvSubGroup
SFVpath
SFVterraine
SFVobjScattering
SFVplatformPose
SFVmass_link
SFVfriction_link
SFVsensor_link
SFVobsOnPathScattering
RULE_platform_init_pose_with_no_obj_colisions
RULE_wp_path_inside_map
GazeboExecutor
GazeboScenarioGenerator
)

##############
# GENERATORS #
##############

add_library(GazeboEnvironmentGenerator src/Generators/Gazebo/GazeboEnvironmentGenerator.cpp)
target_link_libraries(GazeboEnvironmentGenerator ${catkin_LIBRARIES}
ResourceHandler
TerrainAnalyzer
SFVobstacaleOnPath
SFVwp
SFVobsOnPathScattering
)

add_library(GazeboMissionGenerator src/Generators/Gazebo/GazeboMissionGenerator.cpp)
target_link_libraries(GazeboMissionGenerator ${catkin_LIBRARIES}
SFVpath
SFVwp
)

add_library(GazeboPlatformGenerator src/Generators/Gazebo/GazeboPlatformGenerator.cpp)
target_link_libraries(GazeboPlatformGenerator ${catkin_LIBRARIES}
ResourceHandler
SFVmass_link
/usr/lib/x86_64-linux-gnu/libsdformat.so #${SDF_LIBRARIES}
)


add_library(GazeboScenarioGenerator src/Generators/Gazebo/GazeboScenarioGenerator.cpp)
target_link_libraries(GazeboScenarioGenerator ${catkin_LIBRARIES}
GazeboEnvironmentGenerator
GazeboMissionGenerator
GazeboPlatformGenerator
)



############
# Executor #
############
add_library(scenarioLauncher src/Executor/scenarioLauncher.cpp) 
target_link_libraries(scenarioLauncher ${catkin_LIBRARIES} ${PYTHON_LIBRARY}
)

add_library(GazeboExecutor src/Executor/GazeboExecutor.cpp)
target_link_libraries(GazeboExecutor ${catkin_LIBRARIES}
scenarioLauncher
)




#############
#### MAIN ###
#############
add_executable(MainTest src/MainTest.cpp)
target_link_libraries(MainTest ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
ScenarioFeatureGroup
ScenarioFeature
SFDPobj
SFV
#GazeboScenarioGenerator
#GazeboExecutor
)

add_executable(mainGen src/Main.cpp)
target_link_libraries(mainGen ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
ScenarioFeatureGroup
ScenarioFeature
SFDPobj
SFV
GazeboScenarioGenerator
)


######################
#### Replay Player ###
######################
add_executable(ReplayPlayer src/ReplayPlayer/ReplayPlayer.cpp)
target_link_libraries(ReplayPlayer ${catkin_LIBRARIES}
)



###############
#### grader ###
###############
add_executable(grader_node src/Grader/grader_node.cpp)
target_link_libraries(grader_node ${catkin_LIBRARIES}
SFV
ResourceHandler
/usr/lib/x86_64-linux-gnu/libsdformat.so #${SDF_LIBRARIES}
)


