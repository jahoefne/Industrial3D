cmake_minimum_required(VERSION 3.3)
project(Industrial3D)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        data/cone.xyz
        data/bunny.xyz
        main.cpp
        K3DTree.h
        PointCloud.cpp
        PointCloud.h
        Point3D.h
)

add_executable(Industrial3D ${SOURCE_FILES})
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(Industrial3D ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES})
