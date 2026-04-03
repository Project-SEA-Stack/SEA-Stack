#ifndef SEASTACK_TEST_MACROS_H_
#define SEASTACK_TEST_MACROS_H_

#include <cmath>
#include <iostream>
#include <string>

struct TestResults {
    int passed = 0;
    int failed = 0;

    void Summary() const {
        std::cout << "\n========================================" << std::endl;
        std::cout << "Results: " << passed << " passed, " << failed << " failed"
                  << std::endl;
        std::cout << "========================================" << std::endl;
    }
};

#define TEST_ASSERT(condition, message)                                  \
    do {                                                                 \
        if (!(condition)) {                                              \
            std::cerr << "FAILED: " << (message) << std::endl;          \
            std::cerr << "  at " << __FILE__ << ":" << __LINE__         \
                      << std::endl;                                      \
            ++test_results.failed;                                       \
        } else {                                                         \
            ++test_results.passed;                                       \
        }                                                                \
    } while (0)

#define TEST_NEAR(actual, expected, tolerance, message)                  \
    do {                                                                 \
        double _actual = (actual);                                       \
        double _expected = (expected);                                   \
        double _diff = std::abs(_actual - _expected);                    \
        if (_diff > (tolerance)) {                                       \
            std::cerr << "FAILED: " << (message) << std::endl;          \
            std::cerr << "  Expected: " << _expected                     \
                      << ", Got: " << _actual                            \
                      << ", Diff: " << _diff                             \
                      << ", Tolerance: " << (tolerance) << std::endl;    \
            std::cerr << "  at " << __FILE__ << ":" << __LINE__         \
                      << std::endl;                                      \
            ++test_results.failed;                                       \
        } else {                                                         \
            ++test_results.passed;                                       \
        }                                                                \
    } while (0)

#define TEST_ASSERT_NEAR TEST_NEAR

#endif  // SEASTACK_TEST_MACROS_H_
