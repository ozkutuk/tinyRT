cmake_minimum_required(VERSION 3.6)
project(tinyRT C CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic -O3")
endif()

# Source files
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(SOURCES ${SRC_DIR}/main.cpp 
            ${SRC_DIR}/parser.cpp
            ${SRC_DIR}/ppmimage.cpp
            ${SRC_DIR}/tinyxml2.cpp
            ${SRC_DIR}/tinymath.cpp
            ${SRC_DIR}/parser.h
            ${SRC_DIR}/tinyxml2.h
            ${SRC_DIR}/tinymath.h
            ${SRC_DIR}/ppmimage.h)


add_executable(raytracer ${SOURCES})
target_link_libraries(raytracer ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(raytracer PRIVATE "${SRC_DIR}")

