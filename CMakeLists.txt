project(sdls)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/")

FILE(GLOB SRC_FILES *.cpp)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

add_executable(sdls ${SRC_FILES})

target_link_libraries(sdls glut glui GL GLU orocos-kdl)

install(TARGETS sdls RUNTIME DESTINATION bin)
