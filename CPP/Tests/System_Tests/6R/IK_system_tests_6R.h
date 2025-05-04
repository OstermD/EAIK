#ifndef SYSTEM_TESTS_H
#define SYSTEM_TESTS_H

// NR Tests
bool ik_test_7R_KUKA_R800();
bool ik_test_7R_Panda();

// 6R Tests
bool ik_test_puma();
bool ik_test_PUMA_DH();
bool ik_test_IRB6640();
bool ik_test_spherical();
bool ik_test_puma_reversed();
bool ik_test_spherical_reversed();
bool ik_test_IRB6640_reversed();

bool ik_test_234_Parallel();
bool ik_test_123_Parallel();
bool ik_test_123_Parallel_56_Intersecting();
bool ik_test_UR5();
bool ik_test_345_Parallel();
bool ik_test_456_Parallel();
bool ik_test_456_Parallel_12_Intersecting();
bool ik_test_345_Parallel();

bool ik_test_two_Sphericals();

// I/O Component tests
bool test_eigen_IO();
bool test_batched_IK();

// Run all tests
bool run_6R_Tests();
#endif