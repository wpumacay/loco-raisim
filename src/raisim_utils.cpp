
#include <raisim_utils.h>

namespace tysoc {
namespace utils {

    Eigen::Vector3d toEigenVec3( const TVec3& vec )
    {
        return Eigen::Vector3d( vec.x, vec.y, vec.z );
    }

    Eigen::Matrix3d toEigenMat3( const TMat3& mat )
    {
        Eigen::Matrix3d _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res( i, j ) = mat.buff[i + 3 * j];

        return _res;
    }

    TVec3 fromEigenVec3( const Eigen::Vector3d& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TMat3 fromEigenMat3( const Eigen::Matrix3d& mat )
    {
        TMat3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res.buff[i + 3 * j] = mat( i , j );

        return _res;
    }

    raisim::Mesh* createMesh( raisim::World* world, const TShapeData& data, const TBodyData& bodyData )
    {
        return nullptr;
    }

    raisim::Mesh* createEllipsoid( raisim::World* world, const TShapeData& data, const TBodyData& bodyData )
    {
        return nullptr;
    }

    raisim::HeightMap* createHeightmap( raisim::World* world, const TShapeData& data, const TBodyData& bodyData )
    {
        return nullptr;
    }

}}