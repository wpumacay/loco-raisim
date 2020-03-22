#pragma once

#include <loco_common_raisim.h>
#include <primitives/loco_single_body_collider_adapter_raisim.h>
#include <primitives/loco_single_body_adapter.h>

namespace loco {
    class TSingleBody;
}

namespace loco {
namespace raisimlib {

    class TRaisimSingleBodyAdapter : public TISingleBodyAdapter
    {
    public :

        TRaisimSingleBodyAdapter( TSingleBody* body_ref );

        TRaisimSingleBodyAdapter( const TRaisimSingleBodyAdapter& other ) = delete;

        TRaisimSingleBodyAdapter& operator=( const TRaisimSingleBodyAdapter& other ) = delete;

        ~TRaisimSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void OnDetach() override;

        void SetTransform( const TMat4& transform ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void SetForceCOM( const TVec3& force_com ) override;

        void SetTorqueCOM( const TVec3& torque_com ) override;

        void GetTransform( TMat4& dst_transform ) override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        void SetRaisimWorld( raisim::World* raisim_world_ref );

        raisim::SingleBodyObject* raisim_body() { return m_RaisimBodyRef; }

        const raisim::SingleBodyObject* raisim_body() const { return m_RaisimBodyRef; }

    private :

        void _SetupInitialPose();

        void _SetupInitialVelocity();

    private :

        // Reference to the raisim-world, used to create all simulation-related objects
        raisim::World* m_RaisimWorldRef;
        // Reference to-single-object raisim resource (owned by world)
        raisim::SingleBodyObject* m_RaisimBodyRef;
    };

}}