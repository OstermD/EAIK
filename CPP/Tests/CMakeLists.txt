cmake_minimum_required(VERSION 3.9)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../src build_IK)

project(test_EAIK)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package (Eigen3 REQUIRED)

add_executable(Unit_Tests IK_unit_tests.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/kinematic_remodeling.cpp)
target_include_directories(Unit_Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src/IK ${CMAKE_CURRENT_SOURCE_DIR}/../src/IK/utils ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils)
target_link_libraries(Unit_Tests EAIK_Kinematics)
target_link_libraries(Unit_Tests Eigen3::Eigen)

add_executable(System_Tests IK_system_tests.cpp)
target_include_directories(System_Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(System_Tests EAIK)
target_link_libraries(System_Tests Eigen3::Eigen)