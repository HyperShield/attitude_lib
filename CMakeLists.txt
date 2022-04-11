cmake_minimum_required(VERSION 3.5)

project(orientation_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(orientation_lib main.cpp)

target_link_libraries(orientation_lib m)
