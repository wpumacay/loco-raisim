
#include <loco_common_raisim.h>

namespace loco {
namespace raisimlib {

    Eigen::Vector3d vec3_to_eigen( const TVec3& vec )
    {
        return Eigen::Vector3d( vec.x(), vec.y(), vec.z() );
    }

    Eigen::Vector4d vec4_to_eigen( const TVec4& vec )
    {
        return Eigen::Vector4d( vec.x(), vec.y(), vec.z(), vec.w() );
    }

    Eigen::Matrix3d mat3_to_eigen( const TMat3& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        Eigen::Matrix3d eig_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                eig_mat( i, j ) = mat( i, j );
        return eig_mat;
    }

    Eigen::Matrix4d mat4_to_eigen( const TMat4& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        Eigen::Matrix4d eig_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                eig_mat( i, j ) = mat( i, j );
        return eig_mat;
    }

    TVec3 vec3_from_eigen( const Eigen::Vector3d& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TVec4 vec4_from_eigen( const Eigen::Vector4d& vec )
    {
        return TVec4( vec.x(), vec.y(), vec.z(), vec.w() );
    }

    TMat3 mat3_from_eigen( const Eigen::Matrix3d& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat3 tm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    TMat4 mat4_from_eigen( const Eigen::Matrix4d& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat4 tm_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    raisim::Vec<3> vec3_to_raisim( const TVec3& vec )
    {
        raisim::Vec<3> rsm_vec;
        rsm_vec[0] = vec.x();
        rsm_vec[1] = vec.y();
        rsm_vec[2] = vec.z();
        return rsm_vec;
    }

    raisim::Vec<4> vec4_to_raisim( const TVec4& vec )
    {
        raisim::Vec<4> rsm_vec;
        rsm_vec[0] = vec.x();
        rsm_vec[1] = vec.y();
        rsm_vec[2] = vec.z();
        rsm_vec[3] = vec.w();
        return rsm_vec;
    }

    raisim::Mat<3, 3> mat3_to_raisim( const TMat3& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        raisim::Mat<3, 3> rsm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                rsm_mat( i, j ) = mat( i, j );
        return rsm_mat;
    }

    raisim::Mat<4, 4> mat4_to_raisim( const TMat4& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        raisim::Mat<4, 4> rsm_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                rsm_mat( i, j ) = mat( i, j );
        return rsm_mat;
    }

    TVec3 vec3_from_raisim( const raisim::Vec<3>& vec )
    {
        return TVec3( vec[0], vec[1], vec[2] );
    }

    TVec4 vec4_from_raisim( const raisim::Vec<4>& vec )
    {
        return TVec4( vec[0], vec[1], vec[2], vec[3] );
    }

    TMat3 mat3_from_raisim( const raisim::Mat<3, 3>& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat3 tm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    TMat4 mat4_from_raisim( const raisim::Mat<4, 4>& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat4 tm_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    raisim::SingleBodyObject* CreateSingleBody( raisim::World* raisim_world,
                                                const TShapeData& shape_data,
                                                const TInertialData& inertia_data )
    {
        auto mass = ( inertia_data.mass < loco::EPS ) ? 
                        loco::DEFAULT_DENSITY * loco::ComputeVolumeFromShape( shape_data ) : inertia_data.mass;
        switch ( shape_data.type )
        {
            case eShapeType::BOX :
            {
                return raisim_world->addBox( shape_data.size.x(), shape_data.size.y(), shape_data.size.z(), mass );
            }
            case eShapeType::PLANE :
            {
                return raisim_world->addGround();
            }
            case eShapeType::SPHERE :
            {
                return raisim_world->addSphere( shape_data.size.x(), mass );
            }
            case eShapeType::CYLINDER :
            {
                return raisim_world->addCylinder( shape_data.size.x(), shape_data.size.y(), mass );
            }
            case eShapeType::CAPSULE :
            {
                return raisim_world->addCapsule( shape_data.size.x(), shape_data.size.y(), mass );
            }
            case eShapeType::ELLIPSOID :
            {
                return CreateEllipsoid( raisim_world, shape_data.size, mass, ComputeEllipsoidInertia( mass, shape_data.size ) );
            }
            case eShapeType::MESH :
            {
                raisim::Mat<3, 3> inertia;
                if ( ( inertia_data.ixx > loco::EPS ) && ( inertia_data.iyy > loco::EPS ) && 
                     ( inertia_data.izz > loco::EPS ) && ( inertia_data.ixy > -loco::EPS ) && 
                     ( inertia_data.ixz > -loco::EPS ) && ( inertia_data.iyz > -loco::EPS ) )
                {
                    inertia(0, 0) = inertia_data.ixx; inertia(0, 1) = inertia_data.ixy; inertia(0, 2) = inertia_data.ixz;
                    inertia(1, 0) = inertia_data.ixy; inertia(1, 1) = inertia_data.iyy; inertia(1, 2) = inertia_data.iyz;
                    inertia(2, 0) = inertia_data.ixz; inertia(2, 1) = inertia_data.iyz; inertia(2, 2) = inertia_data.izz;
                }
                else
                {
                    inertia = ComputeMeshAABBInertia( mass, shape_data.size.x(), shape_data.mesh_data );
                }
                return CreateMesh( raisim_world, shape_data.size, shape_data.mesh_data, mass, inertia );
            }
            case eShapeType::HFIELD :
            {
                return CreateHfield( raisim_world, shape_data.size, shape_data.hfield_data );
            }
        }

        LOCO_CORE_ERROR( "CreateSingleBody >>> unsupported shape type" );
        return nullptr;
    }

    raisim::Mesh* CreateEllipsoid( raisim::World* raisim_world, const TVec3& half_extents, double mass, const raisim::Mat<3, 3>& inertia )
    {
        const std::string mesh_filepath = "./ellipsoid.stl";
        const raisim::Vec<3> mesh_com = { 0.0, 0.0, 0.0 };
        const double mesh_scale = 1.0;

        //// SaveMeshToStlOnDisk( mesh_filepath, vertices, faces );
        return raisim_world->addMesh( mesh_filepath, mass, inertia, mesh_com, mesh_scale );
    }

    raisim::Mesh* CreateMesh( raisim::World* raisim_world, const TVec3& scale, const TMeshData& mesh_data, double mass, const raisim::Mat<3, 3>& inertia )
    {
        const raisim::Vec<3> mesh_com = { 0.0, 0.0, 0.0 };
        const double mesh_scale = scale.x();
        if ( std::abs( scale.x() - scale.y() ) > 1e-3 || std::abs( scale.y() - scale.z() ) > 1e-3 )
            LOCO_CORE_WARN( "CreateMesh >>> support for mesh-scale is only single-valued, can't set different scale factor for xyz" );

        if ( mesh_data.filename != "" )
        {
            const std::string mesh_filepath = mesh_data.filename;
            return raisim_world->addMesh( mesh_filepath, mass, inertia, mesh_com, mesh_scale );
        }
        else if ( mesh_data.vertices.size() > 0 && mesh_data.faces.size() > 0 )
        {
            const std::string mesh_filepath = "./user_mesh.stl";
            SaveMeshToStlOnDisk( mesh_filepath, mesh_data.vertices, mesh_data.faces );
            return raisim_world->addMesh( mesh_filepath, mass, inertia, mesh_com, mesh_scale );
        }

        LOCO_CORE_ERROR( "CreateMesh >>> given mesh has no filename nor user vertex-data" );
        return nullptr;
    }

    raisim::HeightMap* CreateHfield( raisim::World* raisim_world, const TVec3& size, const THeightFieldData& hfield_data )
    {
        const ssize_t nx_samples = hfield_data.nWidthSamples;
        const ssize_t ny_samples = hfield_data.nDepthSamples;
        const double scale_x = size.x();
        const double scale_y = size.y();
        const double scale_height = size.z();
        const double center_x = 0.0;
        const double center_y = 0.0;

        std::vector<double> heights( nx_samples * ny_samples, 0.0 );
        for ( ssize_t i = 0; i < ny_samples; i++ )
            for ( ssize_t j = 0; j < nx_samples; j++ )
                heights[i * nx_samples + j] = hfield_data.heights[i * nx_samples + j] * scale_height;

        return raisim_world->addHeightMap( nx_samples, ny_samples, scale_x, scale_y, center_x, center_y, heights );
    }

    raisim::Mat<3, 3> ComputeEllipsoidInertia( double mass, const TVec3& half_extents )
    {
        return raisim::Mat<3, 3>::getIdentity();
    }

    raisim::Mat<3, 3> ComputeMeshAABBInertia( double mass, double scale, const TMeshData& mesh_data )
    {
        std::pair<TVec3, TVec3> aabb;
        if ( mesh_data.filename != "" )
        {
            aabb = loco::ComputeMeshAABB( mesh_data.filename );
        }
        else if ( mesh_data.vertices.size() > 0 )
        {
            aabb = loco::ComputeMeshAABB( mesh_data.vertices );
        }
        else
        {
            LOCO_CORE_ERROR( "CreateMesh >>> given mesh has no filename nor user vertex-data" );
            return raisim::Mat<3, 3>::getIdentity();
        }

        auto aabbMin = TVec3( aabb.first.x() * scale, aabb.first.y() * scale, aabb.first.z() * scale );
        auto aabbMax = TVec3( aabb.second.x() * scale, aabb.second.y() * scale, aabb.second.z() * scale );

        raisim::Mat<3, 3> inertia_matrix;
        inertia_matrix.setIdentity();

        double lx = std::abs( aabbMax.x() - aabbMin.x() );
        double ly = std::abs( aabbMax.y() - aabbMin.y() );
        double lz = std::abs( aabbMax.z() - aabbMin.z() );
        inertia_matrix(0, 0) = ( mass / 12.0 ) * ( ly * ly + lz * lz );
        inertia_matrix(1, 1) = ( mass / 12.0 ) * ( lx * lx + lz * lz );
        inertia_matrix(2, 2) = ( mass / 12.0 ) * ( lx * lx + ly * ly );

        return inertia_matrix;
    }

    void SaveMeshToStlOnDisk( const std::string& filepath, const std::vector<float>& vertices, const std::vector<int>& faces )
    {

    }

}}