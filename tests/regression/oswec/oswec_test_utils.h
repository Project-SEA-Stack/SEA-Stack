/**
 * @file oswec_test_utils.h
 * @brief Shared vector math helpers for OSWEC regression tests.
 *
 * This header is Chrono-free so it can compile without Chrono include paths.
 */

#ifndef SEASTACK_TESTS_OSWEC_TEST_UTILS_H
#define SEASTACK_TESTS_OSWEC_TEST_UTILS_H

#include <array>
#include <cmath>

#include <seastack/core/math_constants.h>

inline std::array<double, 3> cross(std::array<double, 3> v1, std::array<double, 3> v2) {
    return {v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]};
}

inline double dot(std::array<double, 3> v1, std::array<double, 3> v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline std::array<double, 3> normalize(std::array<double, 3> v) {
    double norm = sqrt(dot(v, v));
    return {v[0] / norm, v[1] / norm, v[2] / norm};
}

inline std::array<double, 3> rotate_vector_3d(std::array<double, 3> vector,
                                              std::array<double, 3> axis,
                                              double angle_in_degrees) {
    double angle_in_radians = angle_in_degrees * (M_PI / 180.0);
    axis = normalize(axis);
    std::array<double, 3> rotated_vector;
    for (int i = 0; i < 3; i++) {
        rotated_vector[i] = vector[i] * cos(angle_in_radians) + cross(axis, vector)[i] * sin(angle_in_radians) +
                            axis[i] * dot(axis, vector) * (1 - cos(angle_in_radians));
    }
    return rotated_vector;
}

inline std::array<double, 3> add_vectors(std::array<double, 3> v1, std::array<double, 3> v2) {
    return {v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]};
}

#endif  // SEASTACK_TESTS_OSWEC_TEST_UTILS_H
