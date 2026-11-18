// Unit tests for SimulationPacer and RunClock (Chrono-free, VSG-free).
// Drives the pacer with a fake clock and checks that simulated time follows
// wall time at the requested speed without ever changing the step size.

#include <seastack/viz_core/simulation_pacer.h>

#include "test_macros.h"

#include <limits>

using seastack::viz::RunClock;
using seastack::viz::SimulationPacer;

namespace {

// Simulate an interactive loop at a fixed frame rate for wall_duration [s]
// and return the simulated time reached. Each step advances sim time by dt.
double RunFakeLoop(SimulationPacer& pacer, double dt, double frame_rate, double wall_duration,
                   int* total_steps = nullptr) {
    double sim = 0.0;
    double wall = 0.0;
    pacer.Reset(sim, wall);
    const double frame = 1.0 / frame_rate;
    int steps_total = 0;
    while (wall < wall_duration - 1e-12) {
        wall += frame;
        const int n = pacer.StepsDue(sim, wall, dt);
        for (int i = 0; i < n; ++i) {
            sim += dt;
        }
        steps_total += n;
    }
    if (total_steps) {
        *total_steps = steps_total;
    }
    return sim;
}

}  // namespace

int main() {
    TestResults test_results;

    // Real time: 10 s of wall time at 60 fps with dt = 0.01 s reaches ~10 s.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(1.0);
        int steps = 0;
        const double sim = RunFakeLoop(pacer, 0.01, 60.0, 10.0, &steps);
        TEST_NEAR(sim, 10.0, 0.01 + 1e-9, "1x tracks wall time within one step");
        TEST_ASSERT(steps >= 999 && steps <= 1000, "1x takes ~1000 steps of dt");
    }

    // Slow motion: 0.1x over 10 s wall reaches ~1 s simulated.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(0.1);
        const double sim = RunFakeLoop(pacer, 0.001, 60.0, 10.0);
        TEST_NEAR(sim, 1.0, 0.001 + 1e-9, "0.1x tracks wall time");
    }

    // Fast forward: 4x over 5 s wall reaches ~20 s simulated.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(4.0);
        const double sim = RunFakeLoop(pacer, 0.005, 60.0, 5.0);
        TEST_NEAR(sim, 20.0, 0.005 + 1e-9, "4x tracks wall time");
    }

    // Large dt in slow motion: some frames take no step at all.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(0.25);
        pacer.Reset(0.0, 0.0);
        TEST_ASSERT(pacer.StepsDue(0.0, 1.0 / 60.0, 0.05) == 0, "no step due before dt/speed elapses");
        TEST_ASSERT(pacer.StepsDue(0.0, 0.2, 0.05) == 1, "one step due after dt/speed");
    }

    // Unlimited: always reports work due; the caller bounds it by wall budget.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(SimulationPacer::kUnlimited);
        TEST_ASSERT(pacer.IsUnlimited(), "speed 0 selects unlimited");
        pacer.Reset(0.0, 0.0);
        TEST_ASSERT(pacer.StepsDue(0.0, 0.0, 0.01) == std::numeric_limits<int>::max(),
                    "unlimited reports max steps");
        pacer.SetSpeed(-3.0);
        TEST_ASSERT(pacer.IsUnlimited(), "negative speed selects unlimited");
    }

    // Not anchored or invalid dt: no steps.
    {
        SimulationPacer pacer;
        TEST_ASSERT(pacer.StepsDue(0.0, 10.0, 0.01) == 0, "unanchored pacer takes no steps");
        pacer.Reset(0.0, 0.0);
        TEST_ASSERT(pacer.StepsDue(0.0, 10.0, 0.0) == 0, "dt = 0 takes no steps");
    }

    // Re-anchoring drops backlog (e.g. after a pause or falling behind).
    {
        SimulationPacer pacer;
        pacer.SetSpeed(1.0);
        pacer.Reset(0.0, 0.0);
        TEST_ASSERT(pacer.StepsDue(0.0, 5.0, 0.01) >= 499, "backlog accumulates while not stepping");
        pacer.Reset(0.0, 5.0);
        TEST_ASSERT(pacer.StepsDue(0.0, 5.0, 0.01) == 0, "reset drops the backlog");
    }

    // Steps due exactly on a frame boundary are not lost to rounding.
    {
        SimulationPacer pacer;
        pacer.SetSpeed(1.0);
        pacer.Reset(0.0, 0.0);
        double sim = 0.0;
        for (int i = 0; i < 3; ++i) sim += 0.1;  // 0.30000000000000004
        TEST_ASSERT(pacer.StepsDue(sim, 0.4, 0.1) == 1, "boundary step survives fp rounding");
    }

    // RunClock: active time excludes gaps; achieved speed over a window.
    {
        RunClock clock;
        TEST_ASSERT(!clock.HasAchievedSpeed(), "no speed before a full window");
        clock.AddActiveInterval(0.0, 0.25, 0.5);
        TEST_ASSERT(!clock.HasAchievedSpeed(), "still no speed after half window");
        clock.AddActiveInterval(10.0, 10.25, 0.5);  // 9.75 s paused gap is not counted
        TEST_NEAR(clock.ActiveWallTime(), 0.5, 1e-12, "active time excludes paused gap");
        TEST_ASSERT(clock.HasAchievedSpeed(), "speed available after a full window");
        TEST_NEAR(clock.AchievedSpeed(), 2.0, 1e-12, "achieved speed = sim / active wall");
        clock.ResetSpeedWindow();
        TEST_ASSERT(!clock.HasAchievedSpeed(), "reset clears achieved speed");
        TEST_NEAR(clock.ActiveWallTime(), 0.5, 1e-12, "reset keeps total active time");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
