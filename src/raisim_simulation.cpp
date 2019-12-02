
#include <raisim_simulation.h>

namespace tysoc
{

    TRaisimSimulation::TRaisimSimulation( TScenario* scenarioPtr )
        : TISimulation( scenarioPtr )
    {
        /* create raisim-world (main factory of raisim-resources) */
        m_raisimWorld = std::unique_ptr<raisim::World>( new raisim::World() );
        m_raisimWorld->setTimeStep( 0.002 );

        /* create required adapters from the resources in the scenario */
        _createBodyAdapters();
        _createAgentAdapters();
        _createTerrainAdapters();
    }

    TRaisimSimulation::~TRaisimSimulation()
    {
        m_raisimWorld = nullptr;
    }

    void TRaisimSimulation::_createBodyAdapters()
    {
        auto _bodies = m_scenarioPtr->getBodies();
        for ( auto _body : _bodies )
        {
            auto _bodyAdapter = new TRaisimBodyAdapter( _body );
            _bodyAdapter->setRaisimWorld( m_raisimWorld.get() );
            _body->setAdapter( _bodyAdapter );
            m_bodyAdapters.push_back( _bodyAdapter );

            auto _colliders = _body->collisions();
            for ( auto _collider : _colliders )
            {
                auto _colliderAdapter = new TRaisimCollisionAdapter( _collider );
                _collider->setAdapter( _colliderAdapter );
                m_collisionAdapters.push_back( _colliderAdapter );
            }
        }
    }

    void TRaisimSimulation::_createAgentAdapters()
    {
        // @wip
    }

    void TRaisimSimulation::_createTerrainAdapters()
    {
        // @wip
    }

    bool TRaisimSimulation::_initializeInternal()
    {
        /* not any extra resource-creation on this side, just notify that we should assemble all */
        for ( auto _bodyAdapter : m_bodyAdapters )
            dynamic_cast< TRaisimBodyAdapter* >( _bodyAdapter )->onFinishedCreatingResources();

        return true;
    }

    void TRaisimSimulation::_preStepInternal()
    {

    }

    void TRaisimSimulation::_simStepInternal()
    {
        double _tstart = m_raisimWorld->getWorldTime();
        while ( m_raisimWorld->getWorldTime() - _tstart < 1. / 60. )
            m_raisimWorld->integrate();
    }

    void TRaisimSimulation::_postStepInternal()
    {

    }

    void TRaisimSimulation::_resetInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr )
    {
        TYSOC_CORE_INFO( "Simulation-Raisim >>> creating simulation" );
        return new TRaisimSimulation( scenarioPtr );
    }

}