#cmake_minimum_required(VERSION 2.8.0)
#project(scriptable_monitor_rqt)
#find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)
#find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
#include_directories(${catkin_INCLUDE_DIRS})
#catkin_package()
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})
#include_directories(include)
#add_library(scriptable_monitor_rqt 
#	src/scriptable_monitor_rqt.cpp	
#	src/plugin_register.cpp
#)
#target_link_libraries(scriptable_monitor_rqt ${QT_LIBRARIES} ${catkin_LIBRARIES})
#find_package(class_loader)
#class_loader_hide_library_symbols(${PROJECT_NAME})


cmake_minimum_required(VERSION 2.8.3)

project(scriptable_monitor_rqt)

MACRO (QT4_WRAP_UI2 outfiles )
  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${outfile}.h) # Here we set output
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI2)


find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
set(rqt_image_view_SRCS
  src/scriptable_monitor_rqt.cpp
)
set(rqt_image_view_HDRS
    include/scriptable_monitor_rqt/scriptable_monitor_rqt.h  
)
set(rqt_image_view_UIS
    ui/ScriptsList.ui
    ui/AddScript.ui
)
set(rqt_image_view_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
catkin_package(
  INCLUDE_DIRS ${rqt_image_view_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()
qt4_wrap_cpp(rqt_image_view_MOCS ${rqt_image_view_HDRS})
QT4_WRAP_UI2(rqt_image_view_UIS_H ${rqt_image_view_UIS})

include_directories(${rqt_image_view_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_image_view_SRCS} ${rqt_image_view_MOCS} ${rqt_image_view_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
# find_package(class_loader)
# class_loader_hide_library_symbols(${PROJECT_NAME})
