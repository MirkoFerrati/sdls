project(sdls)

cmake_minimum_required(VERSION 2.8)

FILE(GLOB SRC_FILES *.cpp)

add_executable(sdls ${SRC_FILES})

target_link_libraries(sdls glut glui GL GLU)

install(TARGETS sdls RUNTIME DESTINATION bin)
