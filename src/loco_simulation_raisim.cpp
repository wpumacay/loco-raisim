
#include <loco_simulation_raisim.h>

namespace loco {
namespace raisimlib {

    TRaisimSimulation::TRaisimSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_backendId = "RAISIM";

        m_raisimWorld = std::make_unique<raisim::World>();
        m_raisimWorld->setTimeStep( 0.002 );
        m_raisimWorld->setGravity( vec3_to_raisim( { 0, 0, -9.81 } ) );

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TRaisimSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TRaisimSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TRaisimSimulation::~TRaisimSimulation()
    {
        m_raisimWorld = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyec TRaisimSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyec TRaisimSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TRaisimSimulation::_InitializeInternal()
    {
        // Collect raisim-resources from the adapters and assemble any required resources
        // @todo: implement-me ...

        LOCO_CORE_TRACE( "Raisim-backend >>> gravity    : {0}", ToString( vec3_from_eigen( m_raisimWorld->getGravity().e() ) ) );
        LOCO_CORE_TRACE( "Raisim-backend >>> time-step  : {0}", std::to_string( m_raisimWorld->getTimeStep() ) );
        LOCO_CORE_TRACE( "Raisim-backend >>> num-objs   : {0}", std::to_string( m_raisimWorld->getObjList().size() ) );

        return true;
    }

    void TRaisimSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TRaisimSimulation::_SimStepInternal()
    {
        const double target_steptime = 1.0 / 60.0;
        const double sim_start = m_raisimWorld->getWorldTime();
        while ( m_raisimWorld->getWorldTime() - sim_start < target_steptime )
            m_raisimWorld->integrate();
    }

    void TRaisimSimulation::_PostStepInternal()
    {
        // @todo: run loco-contact-manager here to grab all detected contacts
    }

    void TRaisimSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::raisimlib::TRaisimSimulation( scenarioRef );
    }

}}