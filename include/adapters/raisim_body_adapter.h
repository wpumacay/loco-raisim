#pragma once

#include <raisim_common.h>
#include <raisim_utils.h>

#include <adapters/body_adapter.h>
#include <adapters/raisim_collision_adapter.h>

namespace tysoc
{

    class TBody;

    class TRaisimBodyAdapter : public TIBodyAdapter
    {

    public :

        TRaisimBodyAdapter( TBody* bodyPtr );

        ~TRaisimBodyAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setPosition( const TVec3& position ) override;

        void setRotation( const TMat3& rotation ) override;

        void setTransform( const TMat4& transform ) override;

        void getPosition( TVec3& dstPosition ) override;

        void getRotation( TMat3& dstRotation ) override;

        void getTransform( TMat4& dstTransform ) override;

        void setRaisimWorld( raisim::World* world ) { m_raisimWorldRef = world; }

        void onFinishedCreatingResources();

        raisim::SingleBodyObject* body() const { return m_raisimBodyRef; }

    private :

        /* reference to the raisim-world, used to create all simulation-related objects */
        raisim::World* m_raisimWorldRef;

        /* reference to-single-object raisim resource (owned by world), used to read/set simulation state */
        raisim::SingleBodyObject* m_raisimBodyRef;

        /* flag to check if using a hfield or not (to handle some discrepancies between hfields and every other primitive) */
        bool m_isHeightfield;
    };

}