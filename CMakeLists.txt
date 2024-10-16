cmake_minimum_required(VERSION 3.10)
project(radar-tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#cmake_policy(SET CMP0144 NEW)

find_package(PkgConfig)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(LIBSERIAL_INCLUDE_DIR "/usr/local/include/libserial")
set(LIBSERIAL_LIBRARY "/usr/local/lib/libserial.so")
include_directories(${LIBSERIAL_INCLUDE_DIR})

find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY gpiod)
include_directories(${GPIOD_INCLUDE_DIR})

add_executable(
    ${PROJECT_NAME}
    source/main.cpp
    source/mmWaveRadar.cpp
    source/mmWaveRadar_imp.cpp
    source/kalmanFilter.cpp
    source/targetObject.cpp
    source/visualizer.cpp
    source/servo.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${PCL_LIBRARIES} 
    ${LIBSERIAL_LIBRARY} 
    ${GPIOD_LIBRARY}
)