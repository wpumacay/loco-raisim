
#include <adapters/raisim_body_adapter.h>

namespace tysoc
{

    TRaisimBodyAdapter::TRaisimBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {
        m_raisimBodyRef = nullptr;
        m_raisimWorldRef = nullptr;
    }

    TRaisimBodyAdapter::~TRaisimBodyAdapter()
    {
        m_raisimBodyRef = nullptr;
        m_raisimWorldRef = nullptr;
    }

    void TRaisimBodyAdapter::build()
    {
        if ( !m_bodyPtr )
            return;

        /* create raisim-body with the body collider */
        auto _collider = m_bodyPtr->collision();
        if ( !_collider )
        {
            TYSOC_CORE_WARN( "Raisim body-adapter >>> skipped creating raisim-object body with no collider" );
            return;
        }

        auto _colliderData = _collider->data();
        auto _colliderType = _colliderData.type;
        double _mass = ( m_bodyPtr->data().inertialData.mass == 0.0f ) ? 
                            tysoc::computeVolumeFromShape( _colliderData ) : 
                            m_bodyPtr->data().inertialData.mass;

        if ( _colliderType == eShapeType::BOX )
            m_raisimBodyRef = m_raisimWorldRef->addBox( _colliderData.size.x,
                                                        _colliderData.size.y,
                                                        _colliderData.size.z, _mass );
        else if ( _colliderType == eShapeType::PLANE )
            m_raisimBodyRef = m_raisimWorldRef->addGround();
        else if ( _colliderType == eShapeType::SPHERE )
            m_raisimBodyRef = m_raisimWorldRef->addSphere( _colliderData.size.x, _mass );
        else if ( _colliderType == eShapeType::CYLINDER )
            m_raisimBodyRef = m_raisimWorldRef->addCylinder( _colliderData.size.x,
                                                             _colliderData.size.y, _mass );
        else if ( _colliderType == eShapeType::CAPSULE )
            m_raisimBodyRef = m_raisimWorldRef->addCapsule( _colliderData.size.x,
                                                            _colliderData.size.y, _mass );
        else if ( _colliderType == eShapeType::ELLIPSOID )
            m_raisimBodyRef = utils::createMesh( m_raisimWorldRef, _colliderData, m_bodyPtr->data() );
        else if ( _colliderType == eShapeType::MESH )
            m_raisimBodyRef = utils::createMesh( m_raisimWorldRef, _colliderData, m_bodyPtr->data() );
        else if ( _colliderType == eShapeType::HFIELD )
            m_raisimBodyRef = utils::createHeightmap( m_raisimWorldRef, _colliderData, m_bodyPtr->data() );

        if ( !m_raisimBodyRef )
        {
            TYSOC_CORE_ERROR( "Raisim body-adapter >>> couldn't create body with shape \"{0}\"", 
                              tysoc::toString( _colliderData.type ) );
            return;
        }

        auto _bodyType = m_bodyPtr->dyntype();
        if ( _bodyType == eDynamicsType::STATIC )
            m_raisimBodyRef->setBodyType( raisim::BodyType::STATIC );
        else if ( _bodyType == eDynamicsType::KINEMATIC )
            m_raisimBodyRef->setBodyType( raisim::BodyType::KINEMATIC );
        else if ( _bodyType == eDynamicsType::DYNAMIC )
            m_raisimBodyRef->setBodyType( raisim::BodyType::DYNAMIC );

        auto _initialPose = m_bodyPtr->tf0();
        m_raisimBodyRef->setPose( utils::toEigenVec3( _initialPose.getPosition() ),
                                  utils::toEigenMat3( _initialPose.getRotation() ) );
    }

    void TRaisimBodyAdapter::reset()
    {
        if ( !m_bodyPtr || !m_raisimBodyRef )
            return;

        auto _initialPose = m_bodyPtr->tf0();
        m_raisimBodyRef->setPose( utils::toEigenVec3( _initialPose.getPosition() ),
                                  utils::toEigenMat3( _initialPose.getRotation() ) );

        // zero lin. and ang. vel.   |----linear---|----angular---|
        m_raisimBodyRef->setVelocity( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
    }

    void TRaisimBodyAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TRaisimBodyAdapter::setPosition( const TVec3& position )
    {
        if ( !m_raisimBodyRef )
            return;

        m_raisimBodyRef->setPosition( utils::toEigenVec3( position ) );
    }

    void TRaisimBodyAdapter::setRotation( const TMat3& rotation )
    {
        if ( !m_raisimBodyRef )
            return;

        m_raisimBodyRef->setOrientation( utils::toEigenMat3( rotation ) );
    }

    void TRaisimBodyAdapter::setTransform( const TMat4& transform )
    {
        if ( !m_raisimBodyRef )
            return;

        m_raisimBodyRef->setPose( utils::toEigenVec3( transform.getPosition() ),
                                  utils::toEigenMat3( transform.getRotation() ) );
    }

    void TRaisimBodyAdapter::getPosition( TVec3& dstPosition )
    {
        if ( !m_raisimBodyRef )
            return;

        dstPosition = utils::fromEigenVec3( m_raisimBodyRef->getPosition() );
    }

    void TRaisimBodyAdapter::getRotation( TMat3& dstRotation )
    {
        if ( !m_raisimBodyRef )
            return;

        dstRotation = utils::fromEigenMat3( m_raisimBodyRef->getRotationMatrix() );
    }

    void TRaisimBodyAdapter::getTransform( TMat4& dstTransform )
    {
        if ( !m_raisimBodyRef )
            return;

        dstTransform.setPosition( utils::fromEigenVec3( m_raisimBodyRef->getPosition() ) );
        dstTransform.setRotation( utils::fromEigenMat3( m_raisimBodyRef->getRotationMatrix() ) );
    }

    void TRaisimBodyAdapter::onFinishedCreatingResources()
    {
        if ( !m_bodyPtr || !m_raisimBodyRef )
            return;

        auto _collider = m_bodyPtr->collision();
        if ( !_collider )
            return;

        auto _colliderAdapter = _collider->adapter();
        if ( !_colliderAdapter )
            return;

        /* pass the ode collider handle to the collider-adapter  */
        dynamic_cast< TRaisimCollisionAdapter* >( _colliderAdapter )->setRaisimOdeGeom( m_raisimBodyRef->getCollisionObject() );
    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TRaisimBodyAdapter( bodyPtr );
    }

}