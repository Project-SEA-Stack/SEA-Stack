// Core/Hydro-style TU: Eigen via SEAStack::Core only. Must not include Chrono.
#include "test_eigen_allocator_abi.hpp"

#include <Eigen/Core>

EigenAbiReport EigenAbiReportCoreTu() {
    EigenAbiReport r;
    r.max_align_bytes = EIGEN_MAX_ALIGN_BYTES;
    r.max_static_align_bytes = EIGEN_MAX_STATIC_ALIGN_BYTES;
    r.default_align_bytes = EIGEN_DEFAULT_ALIGN_BYTES;
    r.malloc_already_aligned = EIGEN_MALLOC_ALREADY_ALIGNED;
    r.ideal_max_align_bytes = EIGEN_IDEAL_MAX_ALIGN_BYTES;
    r.tu_name = "core";
    return r;
}

EigenHeapBlock EigenAllocateCoreTu(std::size_t rows, std::size_t cols) {
    auto* m = new Eigen::MatrixXd(static_cast<Eigen::Index>(rows),
                                  static_cast<Eigen::Index>(cols));
    m->setConstant(1.25);
    (*m)(0, 0) = 3.5;
    return EigenHeapBlock{m, rows, cols};
}

void EigenDestroyCoreTu(EigenHeapBlock block) {
    auto* m = static_cast<Eigen::MatrixXd*>(block.matrix);
    if (!m) {
        return;
    }
    // Touch before delete so a wrong allocator tends to fault here, not later.
    volatile double sink = (*m)(0, 0);
    (void)sink;
    delete m;
}
