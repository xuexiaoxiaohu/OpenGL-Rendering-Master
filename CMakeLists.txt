#set cmake version number
cmake_minimum_required(VERSION 3.4.0)

#set project name
project(OpenGL-Rendering-Eraser-Project)
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON) 
# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Surface
if(NOT SURFACE_ROOT)
        set(SURFACE_ROOT "" CACHE PATH "surface root path")
endif()
if(SURFACE_ROOT)
	set(SURFACE_LIB_DIR ${SURFACE_ROOT}/lib/)
    set(SURFACE_BIN_DIR ${SURFACE_ROOT}/bin/)
    file(GLOB SURFACE_LIB_FILES ${SURFACE_ROOT}/lib/*.lib)
    file(GLOB SURFACE_BIN_FILES ${SURFACE_ROOT}/bin/*.dll)
    set(SURFACE_INC_DIR ${SURFACE_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without SURFACE")
endif(SURFACE_ROOT)
include_directories(${SURFACE_INC_DIR})

# GLUT
if(NOT GLUT_ROOT)
    set(GLUT_ROOT "" CACHE PATH "surface glut root")
endif()
if(GLUT_ROOT)
	set(GLUT_LIB_DIR ${GLUT_ROOT}/lib/)
    set(GLUT_BIN_DIR ${GLUT_ROOT}/bin/)
    file(GLOB GLUT_LIB_FILES ${GLUT_ROOT}/lib/*.lib)
    file(GLOB GLUT_BIN_FILES ${GLUT_ROOT}/bin/*.dll)
    set(GLUT_INC_DIR ${GLUT_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without GLUT")
endif(GLUT_ROOT)
include_directories(${GLUT_INC_DIR})

# OSG
if(NOT OSG_ROOT)
    set(OSG_ROOT "" CACHE PATH "OSG ROOT")
endif()
if(OSG_ROOT)
	set(OSG_LIB_DIR ${OSG_ROOT}/lib/)
    set(OSG_BIN_DIR ${OSG_ROOT}/bin/)
    file(GLOB OSG_LIB_FILES ${OSG_ROOT}/lib/*.lib)
    file(GLOB OSG_BIN_FILES ${OSG_ROOT}/bin/*.dll)
    set(OSG_INC_DIR ${OSG_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without OSG")
endif(OSG_ROOT)
include_directories(${OSG_INC_DIR})

# OpenMesh
if(NOT OpenMesh_ROOT)
    set(OpenMesh_ROOT "" CACHE PATH "OpenMesh ROOT")
endif()
if(OpenMesh_ROOT)
	set(OpenMesh_LIB_DIR ${OpenMesh_ROOT}/lib/)
    set(OpenMesh_BIN_DIR ${OpenMesh_ROOT}/bin/)
    file(GLOB OpenMesh_LIB_FILES ${OpenMesh_ROOT}/lib/*.lib)
    file(GLOB OpenMesh_BIN_FILES ${OpenMesh_ROOT}/bin/*.dll)
    set(OpenMesh_INC_DIR ${OpenMesh_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without OpenMesh")
endif(OpenMesh_ROOT)
include_directories(${OpenMesh_INC_DIR})

# PCL
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# QT
set(CMAKE_AUTOMOC ON) 
find_package(Qt5Widgets)  
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5OpenGLExtensions)
file(GLOB SRC_FILES "./*.cpp" "./*.cxx" "./*.c") 
file(GLOB HEAD_FILES "./*.h" "./*.hpp" "./*.hxx") 
file(GLOB UI_FILES "./UI/*.ui") 
file(GLOB SHADER_FILES "./Shader/*.frag" "./Shader/*.vert") 

qt5_wrap_ui(WRAP_FILES ${UI_FILES})
qt5_add_resources(QRC_FILES ${RCC_FILES})

source_group("Ui" FILES ${UI_FILES})
source_group("Source" FILES ${SRC_FILES})
source_group("Header" FILES ${HEAD_FILES})
source_group("Shader" FILES ${SHADER_FILES})

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEAD_FILES} ${QRC_FILES} ${WRAP_FILES} ${SHADER_FILES})

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::OpenGLExtensions)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries (${PROJECT_NAME} ${SURFACE_LIB_FILES})
target_link_libraries (${PROJECT_NAME} ${GLUT_LIB_FILES})
target_link_libraries (${PROJECT_NAME} ${OSG_LIB_FILES})
target_link_libraries (${PROJECT_NAME} ${OpenMesh_LIB_FILES})

#copy file from source dirtory to binaray dirtory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${PROJECT_BINARY_DIR}/Release)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Shader DESTINATION ${PROJECT_BINARY_DIR}/Release)

