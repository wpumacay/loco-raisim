#pragma onnce

#include <loco_common_raisim.h>
#include <loco_simulation.h>

#include <primitives/loco_single_body_collider_adapter_raisim.h>
#include <primitives/loco_single_body_adapter_raisim.h>

namespace loco {
namespace raisimlib {

    class TRaisimSimulation : public TISimulation
    {
    public :

        TRaisimSimulation( TScenario* scenarioRef );

        TRaisimSimulation( const TRaisimSimulation& other ) = delete;

        TRaisimSimulation& operator=( const TRaisimSimulation& other ) = delete;

        ~TRaisimSimulation();

        raisim::World* raisim_world() { return m_RaisimWorld.get(); }

        const raisim::World* raisim_world() const { return m_RaisimWorld.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

    private :

        void _CollectSingleBodyAdapters();

        //// void _CollectCompoundAdapters();

        //// void _CollectKintreeAdapters();

        //// void _CollectTerrainGeneratorAdapters();

    private :

        std::unique_ptr<raisim::World> m_RaisimWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}