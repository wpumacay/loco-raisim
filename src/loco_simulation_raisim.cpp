
#include <loco_simulation_raisim.h>

namespace loco {
namespace raisimlib {

    TRaisimSimulation::TRaisimSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_backendId = "RAISIM";

        m_RaisimWorld = std::make_unique<raisim::World>(); m_RaisimWorld->setTimeStep( 0.002 );
        m_RaisimWorld->setGravity( vec3_to_raisim( { 0, 0, -9.81 } ) );

        _CollectSingleBodyAdapters();
        //// _CollectCompoundAdapters();
        //// _CollectKintreeAdapters();
        //// _CollectTerrainGeneratorsAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TRaisimSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TRaisimSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TRaisimSimulation::~TRaisimSimulation()
    {
        m_RaisimWorld = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyec TRaisimSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyec TRaisimSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TRaisimSimulation::_CollectSingleBodyAdapters()
    {
        auto single_bodies = m_scenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TRaisimSingleBodyAdapter>( single_body );
            single_body_adapter->SetRaisimWorld( m_RaisimWorld.get() );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_singleBodyAdapters.push_back( std::move( single_body_adapter ) );

            auto collider = single_body->collider();
            LOCO_CORE_ASSERT( collider, "TRaisimSimulation::_CollectSingleBodyAdapters >>> single-body {0} \
                              doesn't have an associated collider", single_body->name() );

            auto collider_adapter = std::make_unique<TRaisimSingleBodyColliderAdapter>( collider );
            collider_adapter->SetRaisimWorld( m_RaisimWorld.get() );
            collider->SetColliderAdapter( collider_adapter.get() );
            m_collisionAdapters.push_back( std::move( collider_adapter ) );
        }
    }

    bool TRaisimSimulation::_InitializeInternal()
    {
        // Collect raisim-resources from the adapters and assemble any required resources
        // @todo: implement-me ...

        LOCO_CORE_TRACE( "Raisim-backend >>> gravity    : {0}", ToString( vec3_from_eigen( m_RaisimWorld->getGravity().e() ) ) );
        LOCO_CORE_TRACE( "Raisim-backend >>> time-step  : {0}", std::to_string( m_RaisimWorld->getTimeStep() ) );
        LOCO_CORE_TRACE( "Raisim-backend >>> num-objs   : {0}", std::to_string( m_RaisimWorld->getObjList().size() ) );

        return true;
    }

    void TRaisimSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TRaisimSimulation::_SimStepInternal()
    {
        const double target_steptime = 1.0 / 60.0;
        const double sim_start = m_RaisimWorld->getWorldTime();
        while ( m_RaisimWorld->getWorldTime() - sim_start < target_steptime )
            m_RaisimWorld->integrate();
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