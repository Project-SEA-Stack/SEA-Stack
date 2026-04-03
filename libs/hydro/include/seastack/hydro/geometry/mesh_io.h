/*********************************************************************
 * @file  mesh_io.h
 * @brief Minimal OBJ mesh loader for TriangleMesh.
 *
 * Handles 'v' (vertex) and 'f' (face) records. Non-triangular faces
 * (quads, etc.) are triangulated via fan decomposition from the first
 * vertex of each face.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_GEOMETRY_MESH_IO_H
#define SEASTACK_HYDRO_GEOMETRY_MESH_IO_H

#include <seastack/hydro/geometry/triangle_mesh.h>
#include <string>

namespace seastack::hydro::geometry {

/**
 * @brief Load a triangle mesh from a Wavefront OBJ file.
 *
 * @param path  Path to the OBJ file
 * @return TriangleMesh with vertices and triangulated faces
 * @throws std::runtime_error if the file cannot be opened or contains no geometry
 */
TriangleMesh LoadObjMesh(const std::string& path);

}  // namespace seastack::hydro::geometry

#endif  // SEASTACK_HYDRO_GEOMETRY_MESH_IO_H
