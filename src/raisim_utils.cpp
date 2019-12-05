
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

    raisim::Mesh* createMesh( raisim::World* world, const TCollisionData& colliderData, const TBodyData& bodyData )
    {
        auto& _inertialData = bodyData.inertialData;
        double _meshMass = 1.0f;
        double _meshScale = colliderData.size.x;
        const raisim::Vec<3> _meshCOM = { 0.0, 0.0, 0.0 };
        raisim::Mat<3, 3> _meshInertia; _meshInertia.setIdentity();

        /* compute inertial properties according to inertial data available */
        if ( _inertialData.mass == 0.0f )
        {
            /* compute inertial properties using AABB approximation of the mesh */
            _meshMass = tysoc::computeVolumeFromShape( colliderData ) * colliderData.density;
            auto _aabb = tysoc::computeMeshAABB( colliderData.filename );
            auto _aabbMin = TVec3( _aabb.first.x * colliderData.size.x, _aabb.first.y * colliderData.size.y, _aabb.first.z * colliderData.size.z );
            auto _aabbMax = TVec3( _aabb.second.x * colliderData.size.x, _aabb.second.y * colliderData.size.y, _aabb.second.z * colliderData.size.z );
            _meshInertia = _computeInertiaMatrixAABB( (float)_meshMass, _aabbMin, _aabbMax );

            TYSOC_CORE_TRACE( "mass: {0}, inertia: [{1},{2},{3},{4},{5},{6}], aabb-min: {7}, aabb-max: {8}", _meshMass,
                              _meshInertia(0, 0), _meshInertia(1, 1), _meshInertia(2, 2),
                              _meshInertia(0, 1), _meshInertia(0, 2), _meshInertia(1, 2),
                              TVec3::toString( _aabbMin ), TVec3::toString( _aabbMax ) );
        }
        else
        {
            /* grab mass from inertial data, and inertia-matrix if given by user (use aabb otherwise) */
            _meshMass = _inertialData.mass;
            if ( _inertialData.ixx != 0.0f && _inertialData.iyy != 0.0f && _inertialData.izz != 0.0f )
            {
                _meshInertia(0, 0) = _inertialData.ixx;
                _meshInertia(1, 1) = _inertialData.iyy;
                _meshInertia(2, 2) = _inertialData.izz;
                _meshInertia(0, 1) = _meshInertia(1, 0) = _inertialData.ixy;
                _meshInertia(0, 2) = _meshInertia(2, 0) = _inertialData.ixz;
                _meshInertia(1, 2) = _meshInertia(2, 1) = _inertialData.iyz;
            }
            else
            {
                auto _aabb = tysoc::computeMeshAABB( colliderData.filename );
                _meshInertia = _computeInertiaMatrixAABB( (float)_meshMass, 
                                                          { _aabb.first.x * colliderData.size.x, 
                                                            _aabb.first.y * colliderData.size.y,
                                                            _aabb.first.z * colliderData.size.z }, 
                                                          { _aabb.second.x * colliderData.size.x,
                                                            _aabb.second.y * colliderData.size.y,
                                                            _aabb.second.z * colliderData.size.z } );
            }
        }

        return world->addMesh( colliderData.filename, _meshMass, _meshInertia, _meshCOM, _meshScale );
    }

    raisim::Mesh* createEllipsoid( raisim::World* world, const TCollisionData& colliderData, const TBodyData& bodyData )
    {
        return nullptr;
    }

    raisim::HeightMap* createHeightmap( raisim::World* world, const TCollisionData& colliderData, const TBodyData& bodyData, const tysoc::TVec3& position )
    {
        auto& _hfdata = colliderData.hdata;
        std::vector< double > _heights;
        for ( size_t i = 0; i < _hfdata.heightData.size(); i++ )
            _heights.push_back( _hfdata.heightData[i] * colliderData.size.z );

        TYSOC_CORE_TRACE( "scale-x: {0}, scale-y: {1}", colliderData.size.x, colliderData.size.y );
        TYSOC_CORE_TRACE( "n-width: {0}, n-depth: {1}", _hfdata.nWidthSamples, _hfdata.nDepthSamples );
        double _scaleX = colliderData.size.x;
        double _scaleY = colliderData.size.y;
        double _centerX = position.x; 
        double _centerY = position.y;
        return world->addHeightMap( _hfdata.nWidthSamples, _hfdata.nDepthSamples, 
                                    _scaleX, _scaleY, _centerX, _centerY, _heights );
    }

    raisim::Mat<3, 3> _computeInertiaMatrixAABB( float mass, const TVec3& aabbMin, const TVec3& aabbMax )
    {
        raisim::Mat<3, 3> _inertiaMat;
        _inertiaMat.setIdentity();

        float _lx = std::abs( aabbMax.x - aabbMin.x );
        float _ly = std::abs( aabbMax.y - aabbMin.y );
        float _lz = std::abs( aabbMax.z - aabbMin.z );
        _inertiaMat(0, 0) = ( mass / 12.0f ) * ( _ly * _ly + _lz * _lz );
        _inertiaMat(1, 1) = ( mass / 12.0f ) * ( _lx * _lx + _lz * _lz );
        _inertiaMat(2, 2) = ( mass / 12.0f ) * ( _lx * _lx + _ly * _ly );

        return _inertiaMat;
    }

}}