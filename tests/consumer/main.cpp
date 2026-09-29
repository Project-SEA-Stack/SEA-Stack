// Out-of-tree downstream-consumer check (Chrono-free).
//
// Mirrors docs/usage/DOWNSTREAM_PROJECT.md: load BEM data, build a HydroModel,
// evaluate forces, then destroy everything, so Eigen buffers allocated inside
// the SEA-Stack libraries are freed by consumer-compiled code (and vice versa).
// On MSVC this runs under the CRT debug heap: a mismatch in Eigen alignment
// settings between the libraries and the consumer fails here with a heap
// assertion instead of corrupting memory silently.
#include <seastack/core/system_state.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro_io/h5_reader.h>

#include <Eigen/Core>

#include <cstdio>
#include <cstdlib>
#include <iostream>

#ifdef _MSC_VER
#include <crtdbg.h>
#endif

int main(int argc, char** argv) {
#ifdef _MSC_VER
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_CHECK_ALWAYS_DF);
    // Report CRT assertions/errors to stderr instead of a modal dialog.
    for (int type : {_CRT_WARN, _CRT_ERROR, _CRT_ASSERT}) {
        _CrtSetReportMode(type, _CRTDBG_MODE_FILE);
        _CrtSetReportFile(type, _CRTDBG_FILE_STDERR);
    }
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    std::printf("consumer EIGEN_MAX_ALIGN_BYTES=%d\n", EIGEN_MAX_ALIGN_BYTES);
    if (argc < 2) {
        std::cerr << "usage: consumer_check <rm3.h5>\n";
        return 2;
    }
    using namespace seastack::hydro;
    for (int iter = 0; iter < 3; ++iter) {
        HydroData data = seastack::hydro_io::H5FileInfo(argv[1], 2).ReadH5Data();

        SeaStateDefinition sea_state;
        sea_state.type = "regular";
        sea_state.amplitude = 0.5;              // H/2 [m]
        sea_state.omega = 2.0 * 3.14159 / 8.0;  // T = 8 s [rad/s]

        HydroModel model = HydroModelBuilder()
                               .FromHydroData(std::move(data))
                               .WithSeaState(sea_state)
                               .EnableHydrostatics()
                               .EnableRadiation()
                               .EnableExcitation()
                               .Build();

        SystemState state;
        state.bodies.resize(2);
        BodyForces forces = model.Evaluate(state, 0.0);
        std::cout << "iter " << iter << ": body0 Fz = " << forces[0].force.z() << " N\n";
    }
#ifdef _MSC_VER
    if (!_CrtCheckMemory()) {
        std::cerr << "CRT heap check FAILED\n";
        return 3;
    }
#endif
    std::cout << "consumer OK\n";
    return 0;
}
