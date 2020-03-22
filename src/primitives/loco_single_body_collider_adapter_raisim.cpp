
#include <primitives/loco_single_body_collider_adapter_raisim.h>

namespace loco {
namespace raisimlib {

    TRaisimSingleBodyColliderAdapter::TRaisimSingleBodyColliderAdapter( TSingleBodyCollider* collider_ref )
        : TISingleBodyColliderAdapter( collider_ref )
    {
        LOCO_CORE_ASSERT( collider_ref, "TRaisimSingleBodyColliderAdapter >>> given collider reference \
                          should be valid (not nullptr)" );

        m_RaisimWorldRef = nullptr;
        m_RaisimBodyRef = nullptr;
        m_RaisimOdeGeom = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TRaisimSingleBodyColliderAdapter {0} @ {1}", m_ColliderRef->name(), loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TRaisimSingleBodyColliderAdapter " << m_ColliderRef->name() << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TRaisimSingleBodyColliderAdapter::~TRaisimSingleBodyColliderAdapter()
    {
        if ( m_ColliderRef )
            m_ColliderRef->DetachSim();
        m_ColliderRef = nullptr;

        m_RaisimWorldRef = nullptr;
        m_RaisimBodyRef = nullptr;
        m_RaisimOdeGeom = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TRaisimSingleBodyColliderAdapter {0} @ {1}", m_ColliderRef->name(), loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TRaisimSingleBodyColliderAdapter " << m_ColliderRef->name() << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TRaisimSingleBodyColliderAdapter::Build()
    {
        // Nothing to build, as resources are created on single-body adapter
    }

    void TRaisimSingleBodyColliderAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyColliderAdapter::Initialize >>> \
                          must have a valid raisim single-body reference (not nullptr) for collider {0}", m_ColliderRef->name() );

        // Grab ODE-geom handle for later usage
        m_RaisimOdeGeom = m_RaisimBodyRef->getCollisionObject();
    }

    void TRaisimSingleBodyColliderAdapter::OnDetach()
    {
        m_Detached = true;
        m_ColliderRef = nullptr;
    }

    void TRaisimSingleBodyColliderAdapter::ChangeSize( const TVec3& new_size )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyColliderAdapter::ChangeElevationData >>> \
                          must have a valid raisim single-body reference (not nullptr) for collider {0}", m_ColliderRef->name() );

        switch ( m_RaisimBodyRef->getObjectType() )
        {
            case raisim::ObjectType::SPHERE :
            {
                const double radius = new_size.x();
                dGeomSphereSetRadius( m_RaisimOdeGeom, radius );
                break;
            }
            case raisim::ObjectType::BOX :
            {
                const double len_x = new_size.x();
                const double len_y = new_size.y();
                const double len_z = new_size.z();
                dGeomBoxSetLengths( m_RaisimOdeGeom, len_x, len_y, len_z );
                break;
            }
            case raisim::ObjectType::CYLINDER :
            {
                const double radius = new_size.x();
                const double length = new_size.y();
                dGeomCylinderSetParams( m_RaisimOdeGeom, radius, length );
                break;
            }
            case raisim::ObjectType::CAPSULE :
            {
                const double radius = new_size.x();
                const double length = new_size.y();
                dGeomCapsuleSetParams( m_RaisimOdeGeom, radius, length );
                break;
            }
            case raisim::ObjectType::MESH :
            {
                LOCO_CORE_WARN( "TRaisimSingleBodyColliderAdapter::ChangeSize >>> mesh shape resizing is not supported" );
                break;
            }
            case raisim::ObjectType::HEIGHTMAP :
            {
                // Collider data already has updated size, so just recompute heights of heightmap
                ChangeElevationData( m_ColliderRef->data().hfield_data.heights );
                break;
            }
        }
    }

    void TRaisimSingleBodyColliderAdapter::ChangeElevationData( const std::vector<float>& heights )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyColliderAdapter::ChangeElevationData >>> \
                          must have a valid raisim single-body reference (not nullptr) for collider {0}", m_ColliderRef->name() );

        if ( m_ColliderRef->shape() != eShapeType::HFIELD )
        {
            LOCO_CORE_ERROR( "TRaisimSingleBodyColliderAdapter::ChangeElevationData >>> tried chaning \
                              elevation data of not hfield shape, for collider {0}", m_ColliderRef->name() );
            return;
        }

        if ( auto raisim_hmap = dynamic_cast<raisim::HeightMap*>( m_RaisimBodyRef ) )
        {
            // @todo: check if heights were accessed by reference in ode-heightmap. If not, use both
            //        dGeomHeightfieldDataBuildDouble and dGeomHeightfieldGetHeightfieldData
            auto& heights_storage = raisim_hmap->getHeightMap();
            if ( heights.size() != heights_storage.size() )
            {
                LOCO_CORE_ERROR( "TRaisimSingleBodyColliderAdapter::ChangeElevationData >>> given hfield \
                                  data's size {0} doesn't match internal buffer size {1}", heights.size(), heights_storage.size() );
                return;
            }

            const double scale_height = m_ColliderRef->size().z();
            for ( ssize_t i = 0; i < heights_storage.size(); i++ )
                heights_storage[i] = heights[i] * scale_height;
        }
    }

    void TRaisimSingleBodyColliderAdapter::ChangeCollisionGroup( int collisionGroup )
    {
        // @todo: remove from API, as should only be set during initialization
    }

    void TRaisimSingleBodyColliderAdapter::ChangeCollisionMask( int collisionMask )
    {
        // @todo: remove from API, as should only be set during initialization
    }

}}