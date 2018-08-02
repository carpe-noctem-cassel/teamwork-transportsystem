cmake_minimum_required(VERSION 2.8.3)
project(rqt_turtlebutler)

set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
)

#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(Qt5Core REQUIRED)
get_target_property(Qt5Core_location Qt5::Core LOCATION)
find_package(Qt5Gui REQUIRED)
get_target_property(Qt5Gui_location Qt5::Gui LOCATION)
find_package(Qt5OpenGL REQUIRED)
get_target_property(Qt5OpenGL_location Qt5::OpenGL LOCATION)
find_package(Qt5Widgets REQUIRED)
get_target_property(Qt5Widgets_location Qt5::Widgets LOCATION)
SET( QT_USE_QTXML TRUE )
SET( QT_WRAP_CPP TRUE )

#include(${QT_USE_FILE})

set(rqt_turtlebutler_SRCS
  src/rqt_turtlebutler/TurtleButler.cpp
)

set(rqt_turtlebutler_HDRS
  include/rqt_turtlebutler/TurtleButler.h
)

set(rqt_turtlebutler_UIS
  src/rqt_turtlebutler/TurtleButler.ui
)

set(rqt_turtlebutler_INCLUDE_DIRECTORIES
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
#  ${CMAKE_CURRENT_BINARY_DIR}
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rqt_turtlebutler_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
#  DEPENDS system_lib
)

catkin_python_setup()

QT5_WRAP_CPP(rqt_turtlebutler_MOCS ${rqt_turtlebutler_HDRS})

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

QT5_WRAP_UI(rqt_turtlebutler_UIS_H ${rqt_turtlebutler_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${rqt_turtlebutler_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_turtlebutler_SRCS} ${rqt_turtlebutler_MOCS} ${rqt_turtlebutler_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Qt5Core_location} ${Qt5Gui_location} ${Qt5OpenGL_location} ${Qt5Widgets_location})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_turtlebutler
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_turtlebutler
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
