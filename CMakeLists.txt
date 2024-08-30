cmake_minimum_required(VERSION 3.10)
project(Joe)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(serial REQUIRED)
include_directories(${serial_INCLUDE_DIRS})

add_executable(
    joe #EXE Name
    source/main.cpp
    source/mmWaveRadar.cpp
)

target_link_libraries(joe ${PCL_LIBRARIES} ${serial_LIBRARIES})