// Unit tests for the overlay colour range (Chrono-free, VSG-free).

#include <seastack/viz_core/color_range.h>

#include "test_macros.h"

#include <limits>

using seastack::viz::ColorRange;
using seastack::viz::ColorRangeMode;

int main() {
    TestResults test_results;

    // Auto: first update snaps to padded peak; min stays 0.
    {
        ColorRange r;
        r.Update(1000.0f);
        TEST_NEAR(r.Min(), 0.0f, 1e-6, "auto min is zero");
        TEST_NEAR(r.Max(), 1100.0f, 1e-3, "auto max = peak + 10% padding");
        r.Update(2000.0f);
        TEST_NEAR(r.Max(), 2200.0f, 1e-3, "auto snaps up to a new peak");
        for (int i = 0; i < ColorRange::kHoldFrames; ++i) {
            r.Update(100.0f);
        }
        TEST_NEAR(r.Max(), 2200.0f, 1e-3, "auto holds the peak for kHoldFrames");
        r.Update(100.0f);
        TEST_ASSERT(r.Max() < 2200.0f && r.Max() > 2190.0f, "auto then decays slowly");
    }

    // Locked: range frozen, updates ignored.
    {
        ColorRange r;
        r.Update(1000.0f);
        r.SetMode(ColorRangeMode::kLocked);
        r.Update(5000.0f);
        TEST_NEAR(r.Max(), 1100.0f, 1e-3, "locked ignores new peaks");
        r.SetMode(ColorRangeMode::kAuto);
        r.Update(5000.0f);
        TEST_NEAR(r.Max(), 5500.0f, 1e-3, "auto resumes from the locked range");
    }

    // Fixed: user range used, invalid ranges rejected.
    {
        ColorRange r;
        r.Update(1000.0f);
        TEST_ASSERT(r.SetFixedRange(200.0f, 800.0f), "valid fixed range accepted");
        TEST_ASSERT(!r.SetFixedRange(800.0f, 200.0f), "max <= min rejected");
        TEST_ASSERT(!r.SetFixedRange(0.0f, std::numeric_limits<float>::infinity()), "non-finite rejected");
        r.SetMode(ColorRangeMode::kFixed);
        r.Update(5000.0f);
        TEST_NEAR(r.Min(), 200.0f, 1e-6, "fixed min used");
        TEST_NEAR(r.Max(), 800.0f, 1e-6, "fixed max used");
        TEST_NEAR(r.Normalize(500.0f), 0.5f, 1e-6, "normalise mid-range");
        TEST_NEAR(r.Normalize(100.0f), 0.0f, 1e-6, "clamped below");
        TEST_NEAR(r.Normalize(900.0f), 1.0f, 1e-6, "clamped above");
        r.SetMode(ColorRangeMode::kAuto);
        TEST_NEAR(r.Max(), 5500.0f, 1e-3, "auto keeps tracking while fixed is shown");
        TEST_NEAR(r.LastPeak(), 5000.0f, 1e-6, "last peak reported");
    }

    // Locking from fixed freezes the fixed range, not a stale auto range.
    {
        ColorRange r;
        r.SetFixedRange(200.0f, 800.0f);
        r.SetMode(ColorRangeMode::kFixed);
        r.Update(5000.0f);
        r.SetMode(ColorRangeMode::kLocked);
        r.Update(9000.0f);
        TEST_NEAR(r.Min(), 200.0f, 1e-6, "locked keeps the shown min");
        TEST_NEAR(r.Max(), 800.0f, 1e-6, "locked keeps the shown max");
    }

    // Reset restarts auto tracking at the next value.
    {
        ColorRange r;
        r.Update(10000.0f);
        r.ResetAuto();
        r.Update(100.0f);
        TEST_NEAR(r.Max(), 110.0f, 1e-3, "reset re-snaps to the current peak");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
