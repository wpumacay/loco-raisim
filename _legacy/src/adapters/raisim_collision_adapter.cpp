
#include <adapters/raisim_collision_adapter.h>

namespace tysoc
{
    TRaisimCollisionAdapter::TRaisimCollisionAdapter( TCollision* collision )
        : TICollisionAdapter( collision )
    {
        m_raisimOdeGeom = NULL;
    }

    TRaisimCollisionAdapter::~TRaisimCollisionAdapter()
    {
        m_raisimOdeGeom = NULL;
    }

    void TRaisimCollisionAdapter::build()
    {
        /* do nothing in this section, we must wait until the body creates the raisim-object
           in order to grab the handle later once we have finished creating all low-level resources */
    }

    void TRaisimCollisionAdapter::reset()
    {
        // nothing required for now, as the functionality exposed in the other methods seems enough
    }

    void TRaisimCollisionAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TRaisimCollisionAdapter::setLocalPosition( const TVec3& position )
    {
        if ( !m_raisimOdeGeom )
            return;
    }

    void TRaisimCollisionAdapter::setLocalRotation( const TMat3& rotation )
    {
        if ( !m_raisimOdeGeom )
            return;
    }

    void TRaisimCollisionAdapter::setLocalTransform( const TMat4& transform )
    {
        if ( !m_raisimOdeGeom )
            return;
    }

    void TRaisimCollisionAdapter::changeSize( const TVec3& size )
    {
        if ( !m_raisimOdeGeom )
            return;
    }

    void TRaisimCollisionAdapter::changeElevationData( const std::vector< float >& heightData )
    {
        if ( !m_raisimOdeGeom )
            return;
    }

}