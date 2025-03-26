cmake_minimum_required(VERSION 3.9)
project(EAIK_Kin)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../external/ik-geo/cpp/subproblems build_sp)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(sources 6R_IK.cpp 5R_IK.cpp 4R_IK.cpp 3R_IK.cpp 2R_IK.cpp 1R_IK.cpp ${CMAKE_CURRENT_SOURCE_DIR}/utils/kinematic_utils.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../utils/kinematic_remodeling.cpp)

add_library(EAIK_Kinematics ${sources})
target_include_directories(EAIK_Kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../utils ${CMAKE_CURRENT_SOURCE_DIR}/../../external/ik-geo/cpp/subproblems)
target_link_libraries(EAIK_Kinematics SUBPROBLEMS)