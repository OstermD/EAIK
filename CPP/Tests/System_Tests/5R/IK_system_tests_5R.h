#ifndef SYSTEM_TESTS_5R_H
#define SYSTEM_TESTS_5R_H

// 5R Tests

// Three Parallel
bool ik_test_5R_123_Parallel();
bool ik_test_5R_345_Parallel();
bool ik_test_5R_234_Parallel();

// Two intersecting
bool ik_test_5R_12_intersecting();
bool ik_test_5R_45_intersecting();

// Three intersecting
bool ik_test_5R_spherical_base();
bool ik_test_5R_spherical_wrist();

// Run all tests
bool run_5R_Tests();
#endif