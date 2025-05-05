#ifndef SYSTEM_TESTS_4R_H
#define SYSTEM_TESTS_4R_H

// 4R Tests
bool ik_test_4R_non_intersecting_parallel();

// Intersecting axes
bool ik_test_4R_34_intersecting();
bool ik_test_4R_12_intersecting();
bool ik_test_4R_23_intersecting();
bool ik_test_4R_spherical_wrist();
bool ik_test_4R_spherical_base();
bool ik_test_4R_12_34_intersecting();

// Parallels axes
bool ik_test_4R_12_parallel();
bool ik_test_4R_34_parallel();
bool ik_test_4R_23_parallel();
bool ik_test_4R_123_parallel();
bool ik_test_4R_234_parallel();
// Run all tests
bool run_4R_Tests();
#endif