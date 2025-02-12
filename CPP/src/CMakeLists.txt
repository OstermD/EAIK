cmake_minimum_required(VERSION 3.9)
project(EAIK_Cpp)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/IK build_IK)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(sources EAIK.cpp ${CMAKE_CURRENT_SOURCE_DIR}/utils/kinematic_remodeling.cpp ${CMAKE_CURRENT_SOURCE_DIR}/IK/utils/kinematic_utils.cpp)

add_library(EAIK ${sources})
target_include_directories(EAIK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/utils ${CMAKE_CURRENT_SOURCE_DIR}/IK ${CMAKE_CURRENT_SOURCE_DIR}/IK/utils)
target_link_libraries(EAIK EAIK_Kinematics)
