/*********************************************************************
 * @file  mesh_io.cpp
 * @brief Minimal OBJ mesh loader.
 *
 * Reads 'v' and 'f' lines. Faces with more than 3 vertices are
 * triangulated via fan decomposition from vertex 0 of the face.
 * Face vertex indices in OBJ are 1-based; converted to 0-based here.
 * Supports vertex/texture/normal index formats (v, v/vt, v/vt/vn, v//vn).
 *********************************************************************/

#include <seastack/hydro/geometry/mesh_io.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace seastack::hydro::geometry {

namespace {

int ParseFaceVertexIndex(const std::string& token) {
    // OBJ face tokens can be: v, v/vt, v/vt/vn, v//vn
    // We only need the vertex index (first number before any slash).
    auto slash_pos = token.find('/');
    std::string idx_str = (slash_pos != std::string::npos)
                          ? token.substr(0, slash_pos)
                          : token;
    return std::stoi(idx_str) - 1;  // OBJ is 1-based -> 0-based
}

}  // namespace

TriangleMesh LoadObjMesh(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error(
            "LoadObjMesh: cannot open file '" + path + "'");
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> tris;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            verts.emplace_back(x, y, z);
        } else if (prefix == "f") {
            std::vector<int> face_indices;
            std::string token;
            while (iss >> token) {
                face_indices.push_back(ParseFaceVertexIndex(token));
            }
            // Fan triangulation from vertex 0
            for (size_t i = 1; i + 1 < face_indices.size(); ++i) {
                tris.emplace_back(face_indices[0],
                                  face_indices[i],
                                  face_indices[i + 1]);
            }
        }
    }

    if (verts.empty()) {
        throw std::runtime_error(
            "LoadObjMesh: file '" + path + "' contains no vertices");
    }
    if (tris.empty()) {
        throw std::runtime_error(
            "LoadObjMesh: file '" + path + "' contains no faces");
    }

    TriangleMesh mesh;
    mesh.vertices.resize(static_cast<int>(verts.size()), 3);
    for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
        mesh.vertices.row(i) = verts[i].transpose();
    }

    mesh.faces.resize(static_cast<int>(tris.size()), 3);
    for (int i = 0; i < static_cast<int>(tris.size()); ++i) {
        mesh.faces.row(i) = tris[i].transpose();
    }

    return mesh;
}

}  // namespace seastack::hydro::geometry
