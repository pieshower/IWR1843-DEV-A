cmake_minimum_required(VERSION 3.10)

# Set the project name
project(HelloWorld)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add an executable
add_executable(main source/main.cpp)

# Link the standard C++ library
target_link_libraries(main stdc++)

