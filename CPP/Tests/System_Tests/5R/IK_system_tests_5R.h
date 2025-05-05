#ifndef SYSTEM_TESTS_5R_H
#define SYSTEM_TESTS_5R_H

// 5R Tests

// Three Parallel
bool ik_test_5R_123_Parallel();
bool ik_test_5R_345_Parallel();
bool ik_test_5R_234_Parallel();
bool ik_test_5R_123_Parallel_45_Parallel();
bool ik_test_5R_345_Parallel_12_Parallel();
bool ik_test_5R_123_Parallel_34_intersecting();

// Two intersecting
bool ik_test_5R_12_intersecting();
bool ik_test_5R_45_intersecting();
bool ik_test_5R_12_45_intersecting();
bool ik_test_5R_23_45_intersecting();
bool ik_test_5R_12_parallel_45_intersecting();
bool ik_test_5R_23_parallel_45_intersecting();
bool ik_test_5R_23_parallel_45_intersecting_2();

// Three intersecting
bool ik_test_5R_spherical_base();
bool ik_test_5R_spherical_wrist();
bool ik_test_5R_spherical_wrist_12_intersecting();
bool ik_test_5R_spherical_base_45_intersecting();

// Two intermediate intersecting and two parallel
bool ik_test_5R_34_intersecting_23_parallel();
bool ik_test_5R_23_intersecting_34_parallel();
bool ik_test_5R_34_intersecting_23_parallel_45_parallel();

// Run all tests
bool run_5R_Tests();
#endif