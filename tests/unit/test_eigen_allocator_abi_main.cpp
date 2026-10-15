// Two-TU Eigen allocator ABI regression test.
//
// Allocates Eigen::MatrixXd in a Core-style TU and destroys it in a
// Chrono-facing TU (and the reverse). Asserts both TUs report the same
// Eigen ABI macros under the SEA-Stack portable contract.
#include "test_eigen_allocator_abi.hpp"

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>

namespace {

std::string FormatReport(const EigenAbiReport& r) {
    std::ostringstream oss;
    oss << r.tu_name
        << " EIGEN_MAX_ALIGN_BYTES=" << r.max_align_bytes
        << " EIGEN_MAX_STATIC_ALIGN_BYTES=" << r.max_static_align_bytes
        << " EIGEN_DEFAULT_ALIGN_BYTES=" << r.default_align_bytes
        << " EIGEN_MALLOC_ALREADY_ALIGNED=" << r.malloc_already_aligned
        << " EIGEN_IDEAL_MAX_ALIGN_BYTES=" << r.ideal_max_align_bytes;
    return oss.str();
}

bool ReportsMatch(const EigenAbiReport& a, const EigenAbiReport& b) {
    return a.max_align_bytes == b.max_align_bytes &&
           a.max_static_align_bytes == b.max_static_align_bytes &&
           a.default_align_bytes == b.default_align_bytes &&
           a.malloc_already_aligned == b.malloc_already_aligned;
}

#if defined(_MSC_VER)
constexpr int kExpectedMaxAlign = 32;
constexpr int kExpectedStaticAlign = 32;
#else
constexpr int kExpectedMaxAlign = 16;
constexpr int kExpectedStaticAlign = 16;
#endif

bool MatchesPlatformContract(const EigenAbiReport& r) {
    return r.max_align_bytes == kExpectedMaxAlign &&
           r.max_static_align_bytes == kExpectedStaticAlign;
}

}  // namespace

int main() {
    const EigenAbiReport core = EigenAbiReportCoreTu();
    const EigenAbiReport chrono = EigenAbiReportChronoTu();

    std::printf("%s\n", FormatReport(core).c_str());
    std::printf("%s\n", FormatReport(chrono).c_str());

    if (!ReportsMatch(core, chrono)) {
        std::fprintf(stderr,
                     "FAIL: Eigen ABI macros differ between Core TU and Chrono TU\n");
        return 1;
    }
    if (!MatchesPlatformContract(core)) {
        std::fprintf(stderr,
                     "FAIL: expected EIGEN_MAX_ALIGN_BYTES=%d and "
                     "EIGEN_MAX_STATIC_ALIGN_BYTES=%d on this platform\n",
                     kExpectedMaxAlign, kExpectedStaticAlign);
        return 1;
    }

    // Core allocates, Chrono destroys.
    {
        EigenHeapBlock block = EigenAllocateCoreTu(64, 64);
        EigenDestroyChronoTu(block);
    }
    // Chrono allocates, Core destroys.
    {
        EigenHeapBlock block = EigenAllocateChronoTu(64, 64);
        EigenDestroyCoreTu(block);
    }

    // Repeat a few times to stress the heap protocol.
    for (int i = 0; i < 8; ++i) {
        EigenHeapBlock a = EigenAllocateCoreTu(17 + i, 9 + i);
        EigenDestroyChronoTu(a);
        EigenHeapBlock b = EigenAllocateChronoTu(11 + i, 13 + i);
        EigenDestroyCoreTu(b);
    }

    std::printf("eigen allocator ABI OK\n");
    return 0;
}
