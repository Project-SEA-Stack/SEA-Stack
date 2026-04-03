#include <seastack/hydro/geometry/mesh_io.h>
#include <seastack/hydro/geometry/triangle_mesh.h>
#include <seastack/adapters/chrono/helper.h>

#include <filesystem>
#include <iostream>

using namespace seastack::hydro::geometry;

int main() {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    struct MeshCase {
        std::string label;
        std::filesystem::path path;
    };

    MeshCase cases[] = {
        {"float (BEM frame)", DATADIR / "demos" / "rm3" / "geometry" / "float.obj"},
        {"plate (BEM frame)", DATADIR / "demos" / "rm3" / "geometry" / "plate.obj"},
    };

    int failures = 0;
    for (const auto& c : cases) {
        std::cout << "--- " << c.label << " ---\n";
        std::cout << "  Path: " << c.path << "\n";
        if (!std::filesystem::exists(c.path)) {
            std::cerr << "  ERROR: file not found\n";
            ++failures;
            continue;
        }
        auto mesh = LoadObjMesh(c.path.lexically_normal().generic_string());
        std::cout << "  Vertices: " << mesh.num_vertices() << "\n";
        std::cout << "  Faces:    " << mesh.num_faces() << "\n";

        auto topo = mesh.AnalyzeTopology();
        std::cout << "  Closed:   " << (topo.is_closed ? "YES" : "NO") << "\n";
        std::cout << "  Boundary edges: " << topo.num_boundary_edges << "\n";
        if (!topo.is_closed) {
            std::cout << "  Boundary z mean:   " << topo.boundary_z_mean << "\n";
            std::cout << "  Boundary z stddev: " << topo.boundary_z_stddev << "\n";
            std::cerr << "  FAIL: mesh is not watertight\n";
            ++failures;
        } else {
            std::cout << "  PASS: mesh is watertight\n";
        }
        std::cout << "\n";
    }

    if (failures > 0) {
        std::cerr << failures << " mesh(es) failed watertight check\n";
        return 1;
    }
    std::cout << "All RM3 meshes are watertight.\n";
    return 0;
}
