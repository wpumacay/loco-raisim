
#pragma once

#include <string>

#ifndef TYSOC_BACKEND_PHYSICS_RAISIM
    #define TYSOC_BACKEND_PHYSICS_RAISIM "../libtysocPhysicsRaisim.so"
#endif

namespace tysoc {
namespace config {

    namespace physics
    {
        const std::string RAISIM = std::string( TYSOC_BACKEND_PHYSICS_RAISIM );
    }

}}