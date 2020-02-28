#pragma once

#include <raisim_common.h>
#include <raisim_utils.h>

#include <adapters/collision_adapter.h>

namespace tysoc
{

    class TCollision;

    class TRaisimCollisionAdapter : public TICollisionAdapter
    {

    public :

        TRaisimCollisionAdapter( TCollision* collision );

        ~TRaisimCollisionAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void changeSize( const TVec3& size ) override;

        void changeElevationData( const std::vector< float >& heightData) override;

        void setRaisimOdeGeom( dGeomID geom ) { m_raisimOdeGeom = geom; }

    private :

        /* ODE handle to the collision object (ptr) */
        dGeomID m_raisimOdeGeom;

    };

}