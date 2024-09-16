cmake_minimum_required(VERSION 3.9)
project(radar-tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(LIBSERIAL_INCLUDE_DIR "/usr/local/include")
set(LIBSERIAL_LIBRARY "/usr/local/lib/libserial.so")
include_directories(${LIBSERIAL_INCLUDE_DIR})

add_executable(
    radar-tracking #EXE Name
    source/main.cpp
    source/mmWaveRadar.cpp
    source/kalmanFilter.cpp
    source/targetObject.cpp
)

target_link_libraries(radar-tracking ${PCL_LIBRARIES} ${LIBSERIAL_LIBRARY})