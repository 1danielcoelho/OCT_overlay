if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()

add_definitions(-DQT_NO_KEYWORDS) #Allows Qt and Boost.signals to be used
cmake_minimum_required(VERSION 2.4.6)

# Allows us to use the debugger
set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ROS msg and srv
#rosbuild_genmsg()
#rosbuild_gensrv()

#Boost
rosbuild_add_boost_directories()

#Qt-ros
#find_package(Qt4 COMPONENTS QtOpenGL QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})
#rosbuild_include(qt_build qt-ros)
#rosbuild_prepare_qt4(QtCore QtGui)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL QtSvg REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

#OpenCV
find_package(OpenCV 2.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#VTK - Comment the first line: It's for a particular work-around at my home
#computer -Daniel
#set(VTK_DIR "/usr/local/lib/vtk-5.8" CACHE PATH directory FORCE)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

#ITK
#find_package(ITK REQUIRED)
#include(${ITK_USE_FILE})
#include_directories(${VTK_INCLUDE_DIRS})

# Finds include files both in the current directory and also
# in the binary directory
set(CMAKE_INCLUDE_CURRENT_DIR ON)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

#Takes .ui, .qrc, .hpp and .cpp files from their own folders
SET(SOURCES
    main.cpp
    form.cpp
    qnode.cpp
    crossbar.cpp
    octinfo.cpp
    )

SET(HEADERS
    crossbar.h
    octinfo.h
    clustering.h
    sliceinteractor.h
    defines.h
    )

SET(HEADERS_TO_MOC
    form.h
    qnode.h
    )

SET(FORMS
    form.ui
    )

SET(RESOURCES
    )

# Processes all the included files correctly
QT4_WRAP_CPP(HEADERS_MOC ${HEADERS_TO_MOC})
QT4_WRAP_UI(FORMS_HEADERS ${FORMS})
QT4_ADD_RESOURCES(RESOURCES_RCC ${RESOURCES})

#Builds and links all the files
rosbuild_add_executable(OCT_overlay
                      ${QT_SOURCES}
                      ${SOURCES}
                      ${HEADERS}
                      ${HEADERS_MOC}
                      ${FORMS_HEADERS}
                      ${RESOURCES_RCC})
target_link_libraries(OCT_overlay
                      ${QT_LIBRARIES}
                      ${VTK_LIBRARIES}
                      )
rosbuild_link_boost(OCT_overlay system signals)
