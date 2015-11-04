cmake_minimum_required(VERSION 3.3)
project(Industrial3D)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        data/cone.xyz
        main.cpp
        PointCloud.cpp
        PointCloud.h
        nbproject/private/configurations.xml
        nbproject/private/launcher.properties
        nbproject/private/Makefile-variables.mk
        nbproject/private/private.xml
        nbproject/configurations.xml
        nbproject/Makefile-Debug.mk
        nbproject/Makefile-impl.mk
        nbproject/Makefile-Release.mk
        nbproject/Makefile-variables.mk
        nbproject/Package-Debug.bash
        nbproject/Package-Release.bash
        nbproject/project.xml)

add_executable(Industrial3D ${SOURCE_FILES})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(Industrial3D ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES})
