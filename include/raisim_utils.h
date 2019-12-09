#pragma once

#include <raisim_common.h>

namespace tysoc {
namespace utils {

    Eigen::Vector3d toEigenVec3( const tysoc::TVec3& vec );
    Eigen::Matrix3d toEigenMat3( const tysoc::TMat3& mat );

    tysoc::TVec3 fromEigenVec3( const Eigen::Vector3d& vec );
    tysoc::TMat3 fromEigenMat3( const Eigen::Matrix3d& mat );

    raisim::Mesh* createMesh( raisim::World* world, const TCollisionData& shapeData, const TBodyData& bodyData );
    raisim::Mesh* createEllipsoid( raisim::World* world, const TCollisionData& shapeData, const TBodyData& bodyData );
    raisim::HeightMap* createHeightmap( raisim::World* world, const TCollisionData& shapeData, const TBodyData& bodyData );

    raisim::Mat<3, 3> _computeInertiaMatrixAABB( float mass, const TVec3& aabbMin, const TVec3& aabbMax );

}}