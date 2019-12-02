#pragma once

#include <raisim_common.h>
#include <simulation_base.h>

#include <adapters/raisim_collision_adapter.h>
#include <adapters/raisim_body_adapter.h>

namespace tysoc 
{

    class TRaisimSimulation : public TISimulation
    {

    public :

        TRaisimSimulation( TScenario* scenarioPtr );
        ~TRaisimSimulation();

    protected :

        bool _initializeInternal() override;
        void _preStepInternal() override;
        void _simStepInternal() override;
        void _postStepInternal() override;
        void _resetInternal() override;

    private : // helper methods

        void _createBodyAdapters();
        void _createAgentAdapters();
        void _createTerrainAdapters();

    private : // backend-specific resources

        std::unique_ptr<raisim::World> m_raisimWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr );

}