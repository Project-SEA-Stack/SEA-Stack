/*********************************************************************
 * @file  submerged_volume.h
 * @brief ISubmergedVolumeCalculator interface and MeshSubmergedVolume.
 *
 * MAIN TYPES:
 *   - SubmergedVolumeResult: volume + centroid output
 *   - ISubmergedVolumeCalculator: strategy interface for submerged volume
 *   - MeshSubmergedVolume: v1 implementation using divergence theorem
 *
 * The interface is the long-term extensibility seam: future geometry
 * methods (analytical, wave-aware) implement ISubmergedVolumeCalculator
 * without modifying the force component.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_GEOMETRY_SUBMERGED_VOLUME_H
#define SEASTACK_HYDRO_GEOMETRY_SUBMERGED_VOLUME_H

#include <seastack/hydro/geometry/triangle_mesh.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

namespace seastack::hydro::geometry {

/**
 * @brief Result of a submerged volume computation.
 */
struct SubmergedVolumeResult {
    double volume;              ///< Submerged volume V_sub [m^3]
    Eigen::Vector3d centroid;   ///< Centre of buoyancy r_CB [m], world frame
};

/**
 * @brief Strategy interface for computing instantaneous submerged volume.
 *
 * Takes a rigid body transform and waterplane elevation and returns the
 * submerged volume and its centroid. Using Eigen::Affine3d avoids baking
 * any specific pose representation (RPY, quaternion) into the geometry API.
 */
class ISubmergedVolumeCalculator {
public:
    virtual ~ISubmergedVolumeCalculator() = default;

    /**
     * @brief Compute submerged volume and centroid.
     *
     * @param body_to_world  Rigid transform from body frame to world frame
     * @param waterplane_z   Z-coordinate of the flat waterplane in world frame
     * @return SubmergedVolumeResult with volume and centroid
     */
    virtual SubmergedVolumeResult Compute(
        const Eigen::Affine3d& body_to_world,
        double waterplane_z) const = 0;
};

/**
 * @brief v1 submerged volume calculator using divergence theorem on a triangle mesh.
 *
 * Requires a closed (watertight) mesh. Open meshes are rejected at construction
 * with a descriptive error message.
 *
 * Algorithm:
 *   1. Transform mesh vertices to world frame.
 *   2. For each triangle, classify vertices vs waterplane and clip if mixed.
 *   3. Accumulate signed tetrahedron volumes and first moments.
 *   4. Close the clipped region via waterplane cap contributions.
 *   5. Return volume and centroid.
 */
class MeshSubmergedVolume : public ISubmergedVolumeCalculator {
public:
    /**
     * @brief Construct from a triangle mesh.
     *
     * Validates that the mesh is closed (watertight). Throws std::runtime_error
     * if the mesh has boundary edges.
     *
     * @param mesh  Triangle mesh in body-local coordinates (CCW outward normals)
     * @throws std::runtime_error if mesh is not watertight
     */
    explicit MeshSubmergedVolume(TriangleMesh mesh);

    SubmergedVolumeResult Compute(
        const Eigen::Affine3d& body_to_world,
        double waterplane_z) const override;

private:
    TriangleMesh mesh_;
};

}  // namespace seastack::hydro::geometry

#endif  // SEASTACK_HYDRO_GEOMETRY_SUBMERGED_VOLUME_H
