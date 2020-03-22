
#include <primitives/loco_single_body_adapter_raisim.h>

namespace loco {
namespace raisimlib {

    TRaisimSingleBodyAdapter::TRaisimSingleBodyAdapter( TSingleBody* body_ref )
        : TISingleBodyAdapter( body_ref )
    {
        LOCO_CORE_ASSERT( body_ref, "TRaisimSingleBodyAdapter >>> given body reference should be valid \
                          (not nullptr)" );

        m_RaisimWorldRef = nullptr;
        m_RaisimBodyRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TRaisimSingleBodyAdapter {0} @ {1}", m_BodyRef->name(), loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TRaisimSingleBodyAdapter " << m_BodyRef->name() << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TRaisimSingleBodyAdapter::~TRaisimSingleBodyAdapter()
    {
        const std::string name = ( m_BodyRef ) ? m_BodyRef->name() : "undefined";

        if ( m_BodyRef )
            m_BodyRef->DetachSim();
        m_BodyRef = nullptr;

        m_RaisimWorldRef = nullptr;
        m_RaisimBodyRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TRaisimSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TRaisimSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TRaisimSingleBodyAdapter::Build()
    {
        auto collider = m_BodyRef->collider();
        LOCO_CORE_ASSERT( collider, "TRaisimSingleBodyAdapter::Build >>> collider of body {0} should \
                          be valid (not nullptr)", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_RaisimWorldRef, "TRaisimSingleBodyAdapter::Build >>> raisim world-reference \
                          required for building a single-object (not nullptr)" );

        m_RaisimBodyRef = CreateSingleBody( m_RaisimWorldRef, collider->data(), m_BodyRef->data().inertia );
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::Build >>> something wen't wrong while \
                          creating a raisim single-body-object for body {0}", m_BodyRef->name() );

        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
            m_RaisimBodyRef->setBodyType( raisim::BodyType::DYNAMIC );
        else
            m_RaisimBodyRef->setBodyType( raisim::BodyType::STATIC );
    }

    void TRaisimSingleBodyAdapter::Initialize()
    {
        auto collider = m_BodyRef->collider();
        LOCO_CORE_ASSERT( collider, "TRaisimSingleBodyAdapter::Initialize >>> collider of body {0} should \
                          be valid (not nullptr)", m_BodyRef->name() );
        if ( auto collider_adapter = dynamic_cast<TRaisimSingleBodyColliderAdapter*>( collider->collider_adapter() ) )
        {
            collider_adapter->SetRaisimBody( m_RaisimBodyRef );
            collider_adapter->Initialize();
        }

        _SetupInitialPose();
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
            _SetupInitialVelocity();
    }

    void TRaisimSingleBodyAdapter::Reset()
    {
        _SetupInitialPose();
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
            _SetupInitialVelocity();
    }

    void TRaisimSingleBodyAdapter::OnDetach()
    {
        m_Detached = true;
        m_BodyRef = nullptr;
    }

    void TRaisimSingleBodyAdapter::_SetupInitialPose()
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::_SetupInitialPose >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() );

        const auto initial_position = vec3_to_eigen( TVec3( m_BodyRef->tf0().col( 3 ) ) );
        const auto initial_rotation = mat3_to_eigen( TMat3( m_BodyRef->tf0() ) );
        m_RaisimBodyRef->setPose( initial_position, initial_rotation );
    }

    void TRaisimSingleBodyAdapter::_SetupInitialVelocity()
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::_SetupInitialVelocity >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() );

        const auto initial_linear_velocity = vec3_to_eigen( m_BodyRef->linear_vel0() );
        const auto initial_angular_velocity = vec3_to_eigen( m_BodyRef->angular_vel0() );
        m_RaisimBodyRef->setVelocity( initial_linear_velocity, initial_angular_velocity );
    }

    void TRaisimSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::SetTransform >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() );

        const auto position = vec3_to_eigen( TVec3( transform.col( 3 ) ) );
        const auto rotation = mat3_to_eigen( TMat3( transform ) );
        m_RaisimBodyRef->setPose( position, rotation );
    }

    void TRaisimSingleBodyAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::SetLinearVelocity >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        const Eigen::Vector3d new_linear_velocity = vec3_to_eigen( linear_vel );
        const Eigen::Vector3d old_angular_velocity = m_RaisimBodyRef->getAngularVelocity();
        m_RaisimBodyRef->setVelocity( new_linear_velocity, old_angular_velocity );
    }

    void TRaisimSingleBodyAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::SetAngularVelocity >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        const Eigen::Vector3d old_linear_velocity = m_RaisimBodyRef->getLinearVelocity();
        const Eigen::Vector3d new_angular_velocity = vec3_to_eigen( angular_vel );
        m_RaisimBodyRef->setVelocity( old_linear_velocity, new_angular_velocity );
    }

    void TRaisimSingleBodyAdapter::SetForceCOM( const TVec3& force_com )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::SetForceCOM >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        m_RaisimBodyRef->setExternalForce( 0, vec3_to_raisim( force_com ) );
    }

    void TRaisimSingleBodyAdapter::SetTorqueCOM( const TVec3& torque_com )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::SetTorqueCOM >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        m_RaisimBodyRef->setExternalTorque( 0, vec3_to_raisim( torque_com ) );
    }

    void TRaisimSingleBodyAdapter::GetTransform( TMat4& dst_transform )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::GetTransform >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        const auto position = vec3_from_eigen( m_RaisimBodyRef->getPosition() );
        const auto rotation = mat3_from_eigen( m_RaisimBodyRef->getRotationMatrix() );
        dst_transform.set( position, 3 );
        dst_transform.set( rotation );
    }

    void TRaisimSingleBodyAdapter::GetLinearVelocity( TVec3& dst_linear_vel )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::GetLinearVelocity >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        dst_linear_vel = vec3_from_eigen( m_RaisimBodyRef->getLinearVelocity() );
    }

    void TRaisimSingleBodyAdapter::GetAngularVelocity( TVec3& dst_angular_vel )
    {
        LOCO_CORE_ASSERT( m_RaisimBodyRef, "TRaisimSingleBodyAdapter::GetAngularVelocity >>> must have \
                          a valid raisim single-body-object reference (not nullptr) for body {0}", m_BodyRef->name() )

        dst_angular_vel = vec3_from_eigen( m_RaisimBodyRef->getAngularVelocity() );
    }
}}