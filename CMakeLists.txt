cmake_minimum_required(VERSION 3.0.0)

# Replace with your project name
project(${PROJECT_NAME} 0.1.0)

# Add any additional include directories here
include_directories(src include lib library)
include_directories(${workspaceFolder}/lib)

# Unity build
file(GLOB_RECURSE SOURCES src/*.cpp src/*.c)

# Add the -g flag for debugging
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})
