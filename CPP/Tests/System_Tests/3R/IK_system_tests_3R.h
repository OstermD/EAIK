#ifndef SYSTEM_TESTS_3R_H
#define SYSTEM_TESTS_3R_H

// 3R Tests
bool ik_test_3R_generic();
bool ik_test_3R_1_2_Parallel();
bool ik_test_3R_2_3_Parallel();
bool ik_test_3R_1_2_intersecting_2_3_parallel();
bool ik_test_3R_1_2_Parallel_2_3_intersecting();
bool ik_test_3R_1_2_intersecting_2_3_intersecting();
bool ik_test_3R_1_2_3_intersecting();
bool ik_test_3R_1_2_3_intersecting_2();
bool ik_test_3R_1_2_3_parallel();
bool ik_test_3R_1_2_3_parallel_2();
bool ik_test_3R_1_2_intersecting();
bool ik_test_3R_2_3_intersecting();
bool ik_test_3R_PUMA_locked_wrist();

// Run all tests
bool run_3R_Tests();
#endif