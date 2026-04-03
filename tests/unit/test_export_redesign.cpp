/*********************************************************************
 * @file  test_export_redesign.cpp
 * @brief Unit tests for the HDF5 export redesign (schema 1.0).
 *
 * Tests:
 *   1. H5Writer: WriteDataset with gzip compression produces readable output
 *   2. H5Writer: WriteDatasetF32 stores float32 data
 *   3. H5Writer: compressed and uncompressed datasets are numerically identical
 *   4. ExportConfig: default values match spec
 *   5. HydroForces: per-component capture via delta snapshot
 *   6. HydroForces: per-component capture sums to total
 *   7. SetupConfig: output block parsing
 *
 * Self-contained — no external data files required.
 *********************************************************************/

#include <seastack/hydro_io/h5_writer.h>
#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/core/force_component.h>
#include <seastack/core/types.h>
#include <seastack/hydro/hydro_forces.h>
#include <seastack/infra/config/yaml_discovery.h>

#include <H5Cpp.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool condition, const std::string& label) {
    if (condition) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label << "\n";
    }
}

// Temp file helper
static std::string TempH5Path(const std::string& tag) {
    auto p = std::filesystem::temp_directory_path() / ("seastack_test_" + tag + ".h5");
    return p.string();
}

static void Cleanup(const std::string& path) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

// ── Test 1: Compressed WriteDataset round-trip ──────────────────────

static void TestCompressedRoundTrip() {
    const std::string path = TempH5Path("compressed_rt");
    {
        seastack::hydro_io::H5Writer writer(path);
        auto root = writer.Root();

        std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
        std::array<hsize_t, 1> dims = {5};
        seastack::hydro_io::WriteOptions opts;
        opts.compression_level = 1;
        root.WriteDataset("compressed_1d", data, dims, opts);

        std::vector<double> data2d = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        std::array<hsize_t, 2> dims2d = {2, 3};
        root.WriteDataset("compressed_2d", data2d, dims2d, opts);
    }
    // Read back
    {
        H5::H5File file(path, H5F_ACC_RDONLY);
        H5::DataSet ds1d = file.openDataSet("compressed_1d");
        std::vector<double> read1d(5);
        ds1d.read(read1d.data(), H5::PredType::NATIVE_DOUBLE);
        Check(read1d[0] == 1.0 && read1d[4] == 5.0,
              "compressed 1D round-trip values");

        H5::DataSet ds2d = file.openDataSet("compressed_2d");
        std::vector<double> read2d(6);
        ds2d.read(read2d.data(), H5::PredType::NATIVE_DOUBLE);
        Check(read2d[0] == 1.0 && read2d[5] == 6.0,
              "compressed 2D round-trip values");

        // Verify the dataset is actually chunked (compression requires it)
        auto plist = ds1d.getCreatePlist();
        int nfilters = plist.getNfilters();
        Check(nfilters > 0, "compressed dataset has at least one filter");
    }
    Cleanup(path);
}

// ── Test 2: Float32 WriteDataset ────────────────────────────────────

static void TestFloat32Storage() {
    const std::string path = TempH5Path("float32");
    {
        seastack::hydro_io::H5Writer writer(path);
        auto root = writer.Root();

        std::vector<double> data = {1.5, 2.5, 3.5};
        std::array<hsize_t, 1> dims = {3};
        seastack::hydro_io::WriteOptions opts;
        opts.compression_level = 0;
        root.WriteDatasetF32("f32_data", data, dims, opts);
    }
    {
        H5::H5File file(path, H5F_ACC_RDONLY);
        H5::DataSet ds = file.openDataSet("f32_data");

        // Check storage type is float32
        H5::DataType dtype = ds.getDataType();
        Check(dtype.getSize() == 4, "float32 dataset has 4-byte elements");

        std::vector<float> readf(3);
        ds.read(readf.data(), H5::PredType::NATIVE_FLOAT);
        Check(std::abs(readf[0] - 1.5f) < 1e-6f && std::abs(readf[2] - 3.5f) < 1e-6f,
              "float32 values are correct");
    }
    Cleanup(path);
}

// ── Test 3: Compressed vs uncompressed numerical identity ───────────

static void TestCompressedNumericalEquivalence() {
    const std::string path = TempH5Path("equiv");
    std::vector<double> data(1000);
    for (size_t i = 0; i < data.size(); ++i) data[i] = std::sin(i * 0.01);
    std::array<hsize_t, 1> dims = {1000};

    {
        seastack::hydro_io::H5Writer writer(path);
        auto root = writer.Root();

        seastack::hydro_io::WriteOptions no_comp;
        no_comp.compression_level = 0;
        root.WriteDataset("uncompressed", data, dims, no_comp);

        seastack::hydro_io::WriteOptions with_comp;
        with_comp.compression_level = 1;
        root.WriteDataset("compressed", data, dims, with_comp);
    }
    {
        H5::H5File file(path, H5F_ACC_RDONLY);
        std::vector<double> unc(1000), comp(1000);
        file.openDataSet("uncompressed").read(unc.data(), H5::PredType::NATIVE_DOUBLE);
        file.openDataSet("compressed").read(comp.data(), H5::PredType::NATIVE_DOUBLE);

        bool identical = true;
        for (size_t i = 0; i < 1000; ++i) {
            if (unc[i] != comp[i]) { identical = false; break; }
        }
        Check(identical, "gzip-compressed data is bit-identical to uncompressed");
    }
    Cleanup(path);
}

// ── Test 4: ExportConfig defaults ───────────────────────────────────

static void TestExportConfigDefaults() {
    seastack::chrono::ExportConfig cfg;
    Check(cfg.level == seastack::chrono::ExportLevel::kStandard,
          "default export level is kStandard");
    Check(cfg.decimation == 1, "default decimation is 1");
    Check(cfg.compression == true, "default compression is true");
    Check(cfg.use_float32 == false, "default precision is float64");
}

// ── Test 5: HydroForces per-component capture ───────────────────────

namespace {

// Minimal test force component that adds a known force per body.
class TestForceComponent : public seastack::hydro::IHydroForceComponent {
  public:
    TestForceComponent(seastack::hydro::HydroComponentType type,
                       Eigen::Vector3d force_per_body)
        : type_(type), force_(force_per_body) {}

    seastack::hydro::HydroComponentType Type() const override { return type_; }

    void Compute(const seastack::hydro::SystemState& /*state*/, double /*time*/,
                 seastack::hydro::BodyForces& inout_forces) override {
        for (auto& gf : inout_forces) {
            gf.force += force_;
        }
    }

  private:
    seastack::hydro::HydroComponentType type_;
    Eigen::Vector3d force_;
};

}  // namespace

static void TestPerComponentCapture() {
    using namespace seastack::hydro;

    std::vector<std::unique_ptr<IHydroForceComponent>> comps;
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kHydrostatics, Eigen::Vector3d(0, 0, 100)));
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kExcitation, Eigen::Vector3d(50, 0, 0)));
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kRadiation, Eigen::Vector3d(0, -30, 0)));

    HydroForces hf(1, std::move(comps));

    SystemState state;
    state.bodies.push_back(BodyState{});

    // Evaluate WITHOUT per-component capture
    BodyForces total_no_capture = hf.Evaluate(state, 0.0, nullptr);
    Check(std::abs(total_no_capture[0].force.x() - 50.0) < 1e-10,
          "total Fx without capture is correct");
    Check(std::abs(total_no_capture[0].force.z() - 100.0) < 1e-10,
          "total Fz without capture is correct");

    // Evaluate WITH per-component capture
    std::vector<ComponentForceRecord> records;
    BodyForces total_with_capture = hf.Evaluate(state, 0.0, &records);

    Check(records.size() == 3, "three component records returned");
    Check(records[0].type == HydroComponentType::kHydrostatics,
          "first record is hydrostatics");
    Check(std::abs(records[0].forces[0].force.z() - 100.0) < 1e-10,
          "hydrostatics Fz delta is 100");
    Check(std::abs(records[1].forces[0].force.x() - 50.0) < 1e-10,
          "excitation Fx delta is 50");
    Check(std::abs(records[2].forces[0].force.y() - (-30.0)) < 1e-10,
          "radiation Fy delta is -30");

    // Total with capture should equal total without capture
    Check(std::abs(total_with_capture[0].force.x() - total_no_capture[0].force.x()) < 1e-10,
          "total Fx unchanged with capture enabled");
    Check(std::abs(total_with_capture[0].force.z() - total_no_capture[0].force.z()) < 1e-10,
          "total Fz unchanged with capture enabled");
}

// ── Test 6: Component sum equals total ──────────────────────────────

static void TestComponentSumEqualsTotal() {
    using namespace seastack::hydro;

    std::vector<std::unique_ptr<IHydroForceComponent>> comps;
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kHydrostatics, Eigen::Vector3d(10, 20, 30)));
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kExcitation, Eigen::Vector3d(-5, 15, -10)));
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kRadiation, Eigen::Vector3d(3, -8, 4)));
    comps.push_back(std::make_unique<TestForceComponent>(
        HydroComponentType::kDamping, Eigen::Vector3d(-1, 2, -3)));

    HydroForces hf(2, std::move(comps));

    SystemState state;
    state.bodies.resize(2);

    std::vector<ComponentForceRecord> records;
    BodyForces total = hf.Evaluate(state, 1.0, &records);

    for (int bi = 0; bi < 2; ++bi) {
        Eigen::Vector3d sum_f = Eigen::Vector3d::Zero();
        for (const auto& rec : records) {
            sum_f += rec.forces[bi].force;
        }
        double err = (sum_f - total[bi].force).norm();
        Check(err < 1e-10,
              "component sum equals total for body " + std::to_string(bi));
    }
}

// ── Test 7: SetupConfig output block parsing ────────────────────────

static void TestSetupConfigOutputParsing() {
    auto tmp = std::filesystem::temp_directory_path() / "seastack_test_setup.yaml";
    {
        std::ofstream f(tmp);
        f << "model_file: test.model.yaml\n"
          << "simulation_file: test.sim.yaml\n"
          << "output_directory: outputs\n"
          << "output:\n"
          << "  level: detailed\n"
          << "  decimation: 10\n"
          << "  precision: float32\n"
          << "  compression: false\n";
    }

    auto cfg = seastack::infra::ParseSetupFile(tmp);
    Check(cfg.has_model_file, "model_file parsed");
    Check(cfg.has_output_config, "output config detected");
    Check(cfg.output_config.level == "detailed", "output level is detailed");
    Check(cfg.output_config.decimation == 10, "decimation is 10");
    Check(cfg.output_config.precision == "float32", "precision is float32");
    Check(cfg.output_config.compression == false, "compression is false");

    std::error_code ec;
    std::filesystem::remove(tmp, ec);
}

// ── Test 8: PTO power sign convention (unit-level) ──────────────────

static void TestPtoPowerSignConvention() {
    // Pure damper: F = -C * vel (C > 0, vel > 0 => F < 0)
    // absorbed_power = -(F * vel) = -((-C*vel) * vel) = C*vel^2 > 0
    double C = 100.0;
    double vel = 2.0;
    double force_mag = -C * vel; // Chrono convention: -C*vel
    double absorbed_power = -(force_mag * vel);
    Check(absorbed_power > 0, "absorbed_power is positive for passive damper");
    Check(std::abs(absorbed_power - C * vel * vel) < 1e-10,
          "absorbed_power equals C*vel^2");
}

// ── Test 9: Energy trapezoidal integration ──────────────────────────

static void TestEnergyTrapezoidalIntegration() {
    // Constant power = 100 W over 10 steps at dt=0.01 => energy = 100*0.1 = 10 J
    double dt = 0.01;
    int steps = 10;
    double constant_power = 100.0;
    double running_energy = 0.0;
    double prev_power = constant_power;

    for (int i = 1; i <= steps; ++i) {
        double power = constant_power;
        running_energy += 0.5 * (power + prev_power) * dt;
        prev_power = power;
    }
    Check(std::abs(running_energy - 10.0) < 1e-10,
          "trapezoidal energy integration of constant power");

    // Linear ramp: power = 0..100 over 100 steps at dt=0.01
    // Exact integral = 0.5 * 100 * 1.0 = 50 J
    running_energy = 0.0;
    prev_power = 0.0;
    int n = 100;
    for (int i = 1; i <= n; ++i) {
        double power = 100.0 * i / n;
        running_energy += 0.5 * (power + prev_power) * dt;
        prev_power = power;
    }
    Check(std::abs(running_energy - 50.0) < 1e-6,
          "trapezoidal energy integration of linear ramp");
}

// ═════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "=== test_export_redesign ===\n";

    TestCompressedRoundTrip();
    TestFloat32Storage();
    TestCompressedNumericalEquivalence();
    TestExportConfigDefaults();
    TestPerComponentCapture();
    TestComponentSumEqualsTotal();
    TestSetupConfigOutputParsing();
    TestPtoPowerSignConvention();
    TestEnergyTrapezoidalIntegration();

    std::cout << "\nPassed: " << g_pass << "  Failed: " << g_fail << "\n";
    return g_fail > 0 ? 1 : 0;
}
