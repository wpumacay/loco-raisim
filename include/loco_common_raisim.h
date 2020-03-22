#pragma once

#include <loco_common.h>
#include <loco_data.h>
// Main Raisim-API
#include <raisim/World.hpp>

namespace loco {
namespace raisimlib {

    // Conversions from and to raisim-math types and tinymath-math types
    Eigen::Vector3d vec3_to_eigen( const TVec3& vec );
    Eigen::Vector4d vec4_to_eigen( const TVec4& vec );
    Eigen::Matrix3d mat3_to_eigen( const TMat3& mat );
    Eigen::Matrix4d mat4_to_eigen( const TMat4& mat );
    TVec3 vec3_from_eigen( const Eigen::Vector3d& vec );
    TVec4 vec4_from_eigen( const Eigen::Vector4d& vec );
    TMat3 mat3_from_eigen( const Eigen::Matrix3d& mat );
    TMat4 mat4_from_eigen( const Eigen::Matrix4d& mat );

    raisim::Vec<3> vec3_to_raisim( const TVec3& vec );
    raisim::Vec<4> vec4_to_raisim( const TVec4& vec );
    raisim::Mat<3, 3> mat3_to_raisim( const TMat3& mat );
    raisim::Mat<4, 4> mat4_to_raisim( const TMat4& mat );
    TVec3 vec3_from_raisim( const raisim::Vec<3>& vec );
    TVec4 vec4_from_raisim( const raisim::Vec<4>& vec );
    TMat3 mat3_from_raisim( const raisim::Mat<3, 3>& mat );
    TMat4 mat4_from_raisim( const raisim::Mat<4, 4>& mat );

    // Creates a raisim-singlebody given user shape-data
    raisim::SingleBodyObject* CreateSingleBody( raisim::World* raisim_world,
                                                const TShapeData& shape_data,
                                                const TInertialData& inertia_data );

    // Creates a mesh object representing an ellipsoid (saves temporary vertex-data to .stl for loading)
    raisim::Mesh* CreateEllipsoid( raisim::World* raisim_world,
                                   const TVec3& half_extents,
                                   double mass,
                                   const raisim::Mat<3, 3>& inertia );

    // Creates a mesh object for the given mesh-data (either loading from file, or from vertex-data)
    raisim::Mesh* CreateMesh( raisim::World* raisim_world,
                              const TVec3& scale,
                              const TMeshData& mesh_data,
                              double mass,
                              const raisim::Mat<3, 3>& inertia );

    // Creates a heightmap object from the given heightfield data
    raisim::HeightMap* CreateHfield( raisim::World* raisim_world,
                                     const TVec3& size, 
                                     const THeightFieldData& hfield_data );

    // Returns the inertia matrix of an ellipsoid with given mass and half-extents
    raisim::Mat<3, 3> ComputeEllipsoidInertia( double mass, const TVec3& half_extents );

    // Returns the inertia matrix for the AABB of a given mesh
    raisim::Mat<3, 3> ComputeMeshAABBInertia( double mass, double scale, const TMeshData& mesh_data );

    // Saves vertex data to a temporary .stl mesh for later loading by raisim
    void SaveMeshToStlOnDisk( const std::string& filepath,
                              const std::vector<float>& vertices,
                              const std::vector<int>& faces );

}}