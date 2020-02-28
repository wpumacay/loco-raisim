#pragma once

#include <loco_common.h>
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

}}