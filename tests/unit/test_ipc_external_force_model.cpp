#include <seastack/external/external_force_model.h>
#include <seastack/external/ipc_external_force_model.h>
#include <seastack/external/protocol.h>

#include "test_macros.h"

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace {

std::string FindMockModulePath(const char* argv0) {
    namespace fs = std::filesystem;
    fs::path self = fs::absolute(fs::path(argv0)).parent_path();
#ifdef _WIN32
    const fs::path candidate = self / "mock_external_module.exe";
#else
    const fs::path candidate = self / "mock_external_module";
#endif
    if (fs::exists(candidate)) {
        return candidate.string();
    }
    const fs::path alt = self / "mock_external_module";
    if (fs::exists(alt)) {
        return alt.string();
    }
    return candidate.string();
}

}  // namespace

int main(int argc, char** argv) {
    TestResults test_results;
    (void)argc;

    // --- Protocol helpers ---
    {
        seastack::external::ExternalInit init;
        init.kind = "pto";
        init.n_inputs = 2;
        init.n_outputs = 1;
        init.dt = 0.01;
        init.config_json = "{\"damping\":50}";
        const std::string req =
            seastack::external::protocol::MakeInitializeRequest(init);
        TEST_ASSERT(req.find("\"op\":\"initialize\"") != std::string::npos,
                    "initialize request contains op");
        TEST_ASSERT(req.find("\"protocol\":1") != std::string::npos,
                    "initialize request contains protocol version");
        TEST_ASSERT(req.find("\"damping\":50") != std::string::npos,
                    "initialize request embeds config JSON");
    }

    {
        const std::string req =
            seastack::external::protocol::MakeEvaluateRequest(
                1.5, 0.01, {0.1, -2.0});
        TEST_ASSERT(req.find("\"op\":\"evaluate\"") != std::string::npos,
                    "evaluate request contains op");
        std::vector<double> in;
        size_t end = 0;
        const size_t arr = req.find('[');
        TEST_ASSERT(
            seastack::external::protocol::DecodeDoubleArray(req, arr, in, end),
            "decode in array");
        TEST_ASSERT(in.size() == 2, "in array size");
        TEST_NEAR(in[0], 0.1, 1e-12, "in[0]");
        TEST_NEAR(in[1], -2.0, 1e-12, "in[1]");
    }

    {
        std::string status;
        std::string message;
        TEST_ASSERT(seastack::external::protocol::ParseStatusReply(
                        "{\"status\":\"ok\",\"out\":[-100]}", status, message),
                    "parse ok status");
        TEST_ASSERT(status == "ok", "status is ok");
        std::vector<double> out;
        TEST_ASSERT(seastack::external::protocol::ParseEvaluateReply(
                        "{\"status\":\"ok\",\"out\":[-100]}", out),
                    "parse out");
        TEST_ASSERT(out.size() == 1, "out size");
        TEST_NEAR(out[0], -100.0, 1e-12, "out[0]");
    }

    // --- Full IPC round-trip with spawned mock module ---
    {
        seastack::external::IpcExternalForceOptions opts;
        opts.command = {FindMockModulePath(argv[0])};
        opts.timeout_ms = 15000;

        seastack::external::IpcExternalForceModel ipc(opts);
        seastack::external::ExternalInit init;
        init.kind = "pto";
        init.n_inputs = 2;
        init.n_outputs = 1;
        init.dt = 0.01;
        init.config_json = "{\"damping\":50}";

        const auto meta = ipc.Initialize(init);
        TEST_ASSERT(meta.name == "MockDamper", "mock name");
        TEST_ASSERT(meta.version == "1.0", "mock version");

        std::vector<double> out;
        ipc.Evaluate(0.0, {0.0, 2.0}, out);
        TEST_ASSERT(out.size() == 1, "ipc out size");
        TEST_NEAR(out[0], -100.0, 1e-9, "F = -c*v");

        ipc.Evaluate(0.01, {0.1, -1.0}, out);
        TEST_NEAR(out[0], 50.0, 1e-9, "F = -c*(-1)");

        ipc.Reset();
        ipc.Commit();
        ipc.Rollback();
        ipc.Shutdown();
        TEST_ASSERT(true, "shutdown completed");
    }

    // --- Error path: module not connecting ---
    {
        bool threw = false;
        try {
            seastack::external::IpcExternalForceOptions opts;
            opts.timeout_ms = 200;
            seastack::external::IpcExternalForceModel ipc(opts);
            seastack::external::ExternalInit init;
            init.kind = "pto";
            init.n_inputs = 2;
            init.n_outputs = 1;
            init.dt = 0.01;
            ipc.Initialize(init);
        } catch (const std::exception&) {
            threw = true;
        }
        TEST_ASSERT(threw, "timeout when no module connects");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
