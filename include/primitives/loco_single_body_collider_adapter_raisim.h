#pragma once

#include <loco_common_raisim.h>
#include <primitives/loco_single_body_collider_adapter.h>

namespace loco {
namespace raisimlib {

    class TRaisimSingleBodyColliderAdapter : public TISingleBodyColliderAdapter
    {
    public :

        TRaisimSingleBodyColliderAdapter( TSingleBodyCollider* collider_ref );

        TRaisimSingleBodyColliderAdapter( const TRaisimSingleBodyColliderAdapter& other ) = delete;

        TRaisimSingleBodyColliderAdapter& operator=( const TRaisimSingleBodyColliderAdapter& other ) = delete;

        ~TRaisimSingleBodyColliderAdapter();

        void Build() override;

        void Initialize() override;

        void OnDetach() override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

        void SetRaisimWorld( raisim::World* raisim_world_ref ) { m_RaisimWorldRef = raisim_world_ref; }

        void SetRaisimBody( raisim::SingleBodyObject* raisim_body_ref ) { m_RaisimBodyRef = raisim_body_ref; }

        dGeomID ode_geom() { return m_RaisimOdeGeom; }

        const dGeomID ode_geom() const { return m_RaisimOdeGeom; }

    private :

        // Reference to the raisim-world, used to create all simulation-related objects
        raisim::World* m_RaisimWorldRef;
        // ODE handle to the collision object (ptr)
        dGeomID m_RaisimOdeGeom;
        // Reference to-single-object raisim resource (owned by world)
        raisim::SingleBodyObject* m_RaisimBodyRef;
    };

}}