cmake_minimum_required(VERSION 3.5)

project(orientation_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(orientation_lib example/main.cpp inc/mat3.h inc/quaternion.h inc/vec3.h inc/explicit_complementary_filter.h example/attitude.h)

target_link_libraries(orientation_lib m)

target_include_directories(orientation_lib PUBLIC /inc)
