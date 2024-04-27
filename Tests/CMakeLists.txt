cmake_minimum_required(VERSION 3.9)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../src/IK build_IK)
project(test_EAIK)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package (Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} IK_tests.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src/IK)
target_link_libraries(${PROJECT_NAME} EAIK)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)