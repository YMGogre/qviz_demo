##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(qviz_demo)
#add
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
# 添加rViz依赖
find_package(catkin REQUIRED COMPONENTS
    rviz
    roscpp
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    map_server
)

include_directories(${catkin_INCLUDE_DIRS})
#add
# find_package(Qt5 REQUIRED Core Widgets)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

set(QT_LIBRARIES Qt5::Widgets)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#remove
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qviz_demo/*.hpp *.h)

#change
set(QT_RESOURCES
    resources/image.qrc
)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

set(CUSTOM_CONTROL_SOURCES
  CustomControl/MyPalette/mypalette.h
  CustomControl/MyPalette/mypalette.cpp
  CustomControl/MyPalette/mypalette.ui
)

set(OTHER_WIDGET_SOURCES
  OtherWidget/AddLayerWidget/addlayer.h
  OtherWidget/AddLayerWidget/addlayer.cpp
  OtherWidget/AddLayerWidget/addlayer.ui
  OtherWidget/AddMapManager/addmapmanager.h
  OtherWidget/AddMapManager/addmapmanager.cpp
  OtherWidget/AddMapManager/addmapmanager.ui
  OtherWidget/GenerateTask/generatetask.h
  OtherWidget/GenerateTask/generatetask.cpp
  OtherWidget/GenerateTask/generatetask.ui
  OtherWidget/SavePoint/savepoint.h
  OtherWidget/SavePoint/savepoint.cpp
  OtherWidget/SavePoint/savepoint.ui
  OtherWidget/ShellSelectWidget/shellselect.h
  OtherWidget/ShellSelectWidget/shellselect.cpp
  OtherWidget/ShellSelectWidget/shellselect.ui
)


##############################################################################
# Binaries
##############################################################################

add_executable(qviz_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${CUSTOM_CONTROL_SOURCES} ${OTHER_WIDGET_SOURCES})
target_link_libraries(qviz_demo ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS qviz_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

