// Shared declarations for the two-TU Eigen allocator ABI regression test.
//
// TU A (Core/Hydro-style): SEAStack::Core only — no Chrono headers.
// TU B (Chrono-facing): Chrono headers + Chrono::Chrono_core link interface.
// Both must share the same Eigen heap protocol so MatrixXd allocations can
// cross the boundary in either direction without double-free / heap corruption.
#pragma once

#include <cstddef>

struct EigenAbiReport {
    int max_align_bytes = 0;
    int max_static_align_bytes = 0;
    int default_align_bytes = 0;
    int malloc_already_aligned = 0;
    int ideal_max_align_bytes = 0;
    const char* tu_name = "";
};

// Opaque heap block owned as Eigen::MatrixXd* in the allocating TU.
struct EigenHeapBlock {
    void* matrix = nullptr;  // Eigen::MatrixXd*
    std::size_t rows = 0;
    std::size_t cols = 0;
};

EigenAbiReport EigenAbiReportCoreTu();
EigenAbiReport EigenAbiReportChronoTu();

EigenHeapBlock EigenAllocateCoreTu(std::size_t rows, std::size_t cols);
void EigenDestroyCoreTu(EigenHeapBlock block);

EigenHeapBlock EigenAllocateChronoTu(std::size_t rows, std::size_t cols);
void EigenDestroyChronoTu(EigenHeapBlock block);
