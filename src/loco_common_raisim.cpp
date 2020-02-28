
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

}}