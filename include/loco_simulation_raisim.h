#pragma onnce

#include <loco_common_raisim.h>
#include <loco_simulation.h>

namespace loco {
namespace raisimlib {

    class TRaisimSimulation : public TISimulation
    {
    public :

        TRaisimSimulation( TScenario* scenarioRef );

        TRaisimSimulation( const TRaisimSimulation& other ) = delete;

        TRaisimSimulation& operator=( const TRaisimSimulation& other ) = delete;

        ~TRaisimSimulation();

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

    private :

        std::unique_ptr<raisim::World> m_raisimWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}