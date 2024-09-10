cmake_minimum_required(VERSION 3.10)
project(radar-tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED)
find_package(serial REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(${serial_INCLUDE_DIRS})

add_executable(
    radar-tracking #EXE Name
    source/main.cpp
    source/mmWaveRadar.cpp
    source/kalmanFilter.cpp
    source/targetObject.cpp
)

target_link_libraries(radar-tracking ${PCL_LIBRARIES} ${serial_LIBRARIES})