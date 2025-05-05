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

add_executable(Unit_Tests IK_unit_tests.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/kinematic_remodeling.cpp ${CMAKE_CURRENT_SOURCE_DIR}/Utils/IK_test_utils.cpp)
target_include_directories(Unit_Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_SOURCE_DIR}/../src/IK ${CMAKE_CURRENT_SOURCE_DIR}/../src/IK/utils ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils ${CMAKE_CURRENT_SOURCE_DIR}/Utils)
target_link_libraries(Unit_Tests EAIK_Kinematics EAIK)
target_link_libraries(Unit_Tests Eigen3::Eigen)


set(system_test_sources /6R/IK_system_tests_6R.cpp /5R/IK_system_tests_5R.cpp /4R/IK_system_tests_4R.cpp /3R/IK_system_tests_3R.cpp /2R/IK_system_tests_2R.cpp /1R/IK_system_tests_1R.cpp)
set(system_test_includes /6R /5R /4R /3R /2R /1R)

list(TRANSFORM system_test_sources
     PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/System_Tests/"
     OUTPUT_VARIABLE prefixed_system_test_sources
)

list(TRANSFORM system_test_includes
     PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/System_Tests/"
     OUTPUT_VARIABLE prefixed_system_test_includes
)

add_executable(System_Tests IK_system_tests.cpp ${CMAKE_CURRENT_SOURCE_DIR}/Utils/IK_test_utils.cpp ${prefixed_system_test_sources})
target_include_directories(System_Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_SOURCE_DIR}/Utils ${prefixed_system_test_includes})
target_link_libraries(System_Tests EAIK)
target_link_libraries(System_Tests Eigen3::Eigen)