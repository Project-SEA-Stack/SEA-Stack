// Chrono-facing TU: Chrono headers + Chrono INTERFACE compile options.
// Links Chrono::Chrono_core so any native-arch / Eigen flags Chrono exports
// apply to this translation unit (the failure mode under a non-portable Chrono).
#include "test_eigen_allocator_abi.hpp"

#include <chrono/core/ChVector3.h>

#include <Eigen/Core>

EigenAbiReport EigenAbiReportChronoTu() {
    // Force a Chrono type into the TU so the Chrono include is not DCE'd away.
    chrono::ChVector3d v(1.0, 2.0, 3.0);
    (void)v;

    EigenAbiReport r;
    r.max_align_bytes = EIGEN_MAX_ALIGN_BYTES;
    r.max_static_align_bytes = EIGEN_MAX_STATIC_ALIGN_BYTES;
    r.default_align_bytes = EIGEN_DEFAULT_ALIGN_BYTES;
    r.malloc_already_aligned = EIGEN_MALLOC_ALREADY_ALIGNED;
    r.ideal_max_align_bytes = EIGEN_IDEAL_MAX_ALIGN_BYTES;
    r.tu_name = "chrono";
    return r;
}

EigenHeapBlock EigenAllocateChronoTu(std::size_t rows, std::size_t cols) {
    auto* m = new Eigen::MatrixXd(static_cast<Eigen::Index>(rows),
                                  static_cast<Eigen::Index>(cols));
    m->setConstant(2.5);
    (*m)(0, 0) = 7.0;
    return EigenHeapBlock{m, rows, cols};
}

void EigenDestroyChronoTu(EigenHeapBlock block) {
    auto* m = static_cast<Eigen::MatrixXd*>(block.matrix);
    if (!m) {
        return;
    }
    volatile double sink = (*m)(0, 0);
    (void)sink;
    delete m;
}
