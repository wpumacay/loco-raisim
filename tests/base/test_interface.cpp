
#include "test_interface.h"


namespace raisim
{

    /***************************************************************************
    *                                                                          *
    *                               UTILITIES                                  *
    *                                                                          *
    ***************************************************************************/

    eShapeType fromRaisimShapeType( raisim::Shape::Type type )
    {
        switch ( type )
        {

            case raisim::Shape::Type::Box       : return eShapeType::BOX;
            case raisim::Shape::Type::Sphere    : return eShapeType::SPHERE;
            case raisim::Shape::Type::Cylinder  : return eShapeType::CYLINDER;
            case raisim::Shape::Type::Capsule   : return eShapeType::CAPSULE;
            case raisim::Shape::Type::Mesh      : return eShapeType::MESH;

            default :
                return eShapeType::SPHERE;
        }
    }

    eJointType fromRaisimJointType( raisim::Joint::Type type )
    {
        switch ( type )
        {

            case raisim::Joint::Type::FIXED       : return eJointType::FIXED;
            case raisim::Joint::Type::REVOLUTE    : return eJointType::REVOLUTE;
            case raisim::Joint::Type::PRISMATIC  : return eJointType::PRISMATIC;
            case raisim::Joint::Type::SPHERICAL   : return eJointType::BALL;
            case raisim::Joint::Type::FLOATING      : return eJointType::FIXED;

            default :
                return eJointType::FIXED;
        }
    }

    std::vector< std::string > collectAvailableModels( const std::string& folderpath )
    {
        DIR* _directoryPtr;
        struct dirent* _direntPtr;

        _directoryPtr = opendir( folderpath.c_str() );
        if ( !_directoryPtr )
            return std::vector< std::string >();

        auto _modelfiles = std::vector< std::string >();

        // grab each .xml mjcf model
        while ( _direntPtr = readdir( _directoryPtr ) )
        {
            std::string _fname = _direntPtr->d_name;
            if ( _fname.find( ".xml" ) == std::string::npos )
                continue;

            _modelfiles.push_back( _fname );
        }
        closedir( _directoryPtr );

        return _modelfiles;
    }

    Eigen::Vector3d toEigenVec3( const tysoc::TVec3& vec )
    {
        return Eigen::Vector3d( vec.x, vec.y, vec.z );
    }

    Eigen::Matrix3d toEigenMat3( const tysoc::TMat3& mat )
    {
        Eigen::Matrix3d _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res( i, j ) = mat.buff[i + 3 * j];

        return _res;
    }

    tysoc::TVec3 fromEigenVec3( const Eigen::Vector3d& vec )
    {
        return tysoc::TVec3( vec.x(), vec.y(), vec.z() );
    }

    tysoc::TMat3 fromEigenMat3( const Eigen::Matrix3d& mat )
    {
        tysoc::TMat3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res.buff[i + 3 * j] = mat( i , j );

        return _res;
    }

    double computeMassFromShape( const ShapeData& shapeData )
    {
        double _volume = 1.0;
        double _pi = 3.141592653589793;

        if ( shapeData.type == eShapeType::BOX )
            _volume = shapeData.size.x * shapeData.size.y * shapeData.size.z;

        else if ( shapeData.type == eShapeType::SPHERE )
        {
            double _sphereRadius = shapeData.size.x;

            _volume = ( 4. / 3. ) * _pi * ( _sphereRadius * _sphereRadius * _sphereRadius );
        }
        else if ( shapeData.type == eShapeType::CYLINDER )
        {
            double _cylinderRadius = shapeData.size.x;
            double _cylinderHeight = shapeData.size.y;

            _volume = ( _pi * _cylinderRadius * _cylinderRadius ) * _cylinderHeight;
        }
        else if ( shapeData.type == eShapeType::CAPSULE )
        {
            double _capsuleRadius = shapeData.size.x;
            double _capsuleHeight = shapeData.size.y;

            _volume = ( _pi * _capsuleRadius * _capsuleRadius ) * _capsuleHeight +
                      ( 4. / 3. ) * ( _pi * _capsuleRadius * _capsuleRadius * _capsuleRadius );
        }
        else
            std::cout << "WARNING> shape of type: " << std::to_string( shapeData.type )
                      << " not supported yet. Using volume = 1.0 as default" << std::endl;

        return _volume * DEFAULT_DENSITY;
    }

    raisim::SingleBodyObject* createSingleBodyObj( const ShapeData& shapeData,
                                                   raisim::World* raisimWorldPtr )
    {
        raisim::SingleBodyObject* _raisimBodyObj = NULL;

        double _mass = computeMassFromShape( shapeData );

        if ( shapeData.type == eShapeType::BOX )
            _raisimBodyObj = raisimWorldPtr->addBox( shapeData.size.x, shapeData.size.y, shapeData.size.z, _mass );

        else if ( shapeData.type == eShapeType::SPHERE )
            _raisimBodyObj = raisimWorldPtr->addSphere( shapeData.size.x, _mass );

        else if ( shapeData.type == eShapeType::CYLINDER )
            _raisimBodyObj = raisimWorldPtr->addCylinder( shapeData.size.x, shapeData.size.y, _mass );

        else if ( shapeData.type == eShapeType::CAPSULE )
            _raisimBodyObj = raisimWorldPtr->addCapsule( shapeData.size.x, shapeData.size.y, _mass );

        else
            std::cout << "ERROR> shape of type: " << std::to_string( shapeData.type ) << " not supported" << std::endl;

        return _raisimBodyObj;
    }

    engine::LIRenderable* createRenderableShape( const ShapeData& shapeData,
                                                 engine::LScene* graphicsScene )
    {
        engine::LIRenderable* _renderablePtr = NULL;

        if ( shapeData.type == eShapeType::BOX )
        {
            _renderablePtr = engine::LMeshBuilder::createBox( shapeData.size.x,
                                                              shapeData.size.y,
                                                              shapeData.size.z );
        }
        else if ( shapeData.type == eShapeType::SPHERE )
        {
            _renderablePtr = engine::LMeshBuilder::createSphere( shapeData.size.x );
        }
        else if ( shapeData.type == eShapeType::CYLINDER )
        {
            _renderablePtr = engine::LMeshBuilder::createCylinder( shapeData.size.x,
                                                                   shapeData.size.y );
        }
        else if ( shapeData.type == eShapeType::CAPSULE )
        {
            _renderablePtr = engine::LMeshBuilder::createCapsule( shapeData.size.x,
                                                                  shapeData.size.y );
        }

        if ( !_renderablePtr )
            return NULL;

        _renderablePtr->getMaterial()->setColor( { shapeData.color.x, shapeData.color.y, shapeData.color.z } );

        graphicsScene->addRenderable( _renderablePtr );

        return _renderablePtr;
    }

    /***************************************************************************
    *                                                                          *
    *                             BODY WRAPPER                                 *
    *                                                                          *
    ***************************************************************************/

    SimBody::SimBody( const std::string& name )
    {
        m_name = name;
        m_isAlive = false;
        m_graphicsScene = NULL;
        m_graphicsObj = NULL;
        m_raisimWorldPtr = NULL;
        m_raisimBodyPtr = NULL;
    }

    void SimBody::build( raisim::World* raisimWorldPtr,
                         engine::LScene* graphicsScene,
                         const ShapeData& shapeData,
                         bool isFree )
    {
        // cache the reference for later usage
        m_raisimWorldPtr = raisimWorldPtr;
        m_graphicsScene = graphicsScene;

        // create both physics and graphics resources
        m_raisimBodyPtr = createSingleBodyObj( shapeData, m_raisimWorldPtr );
        m_graphicsObj   = createRenderableShape( shapeData, m_graphicsScene );

        // set body-type accordingly (just static or dynamics for now)
        m_raisimBodyPtr->setBodyType( isFree ? raisim::BodyType::DYNAMIC : raisim::BodyType::STATIC );

        // make sure this object is marked as alive for other to not delete it
        m_isAlive = true;
    }

    void SimBody::update()
    {
        if ( !m_raisimBodyPtr )
            return;

        m_worldPos = fromEigenVec3( m_raisimBodyPtr->getPosition() );
        m_worldRot = fromEigenMat3( m_raisimBodyPtr->getRotationMatrix() );

        if ( m_graphicsObj )
        {
            m_graphicsObj->pos = { m_worldPos.x, m_worldPos.y, m_worldPos.z };

            for ( size_t i = 0; i < 3; i++ )
                for ( size_t j = 0; j < 3; j++ )
                    m_graphicsObj->rotation.set( i, j, m_worldRot.buff[i + 3 * j] );
        }
    }

    void SimBody::setPosition( const tysoc::TVec3& position )
    {
        if ( !m_raisimBodyPtr )
            return;

        m_worldPos = position;
        m_raisimBodyPtr->setPosition( toEigenVec3( position ) );
    }

    void SimBody::setOrientation( const tysoc::TMat3& orientation )
    {
        if ( !m_raisimBodyPtr )
            return;

        m_worldRot = orientation;
        m_raisimBodyPtr->setOrientation( toEigenMat3( orientation ) );
    }

    /***************************************************************************
    *                                                                          *
    *                             AS-Link WRAPPER                              *
    *                                                                          *
    ***************************************************************************/

    SimLink::SimLink( const std::string& name,
                      raisim::ArticulatedSystem* raisimASystemPtr,
                      engine::LScene* graphicsScene )
    {
        m_name = name;
        m_raisimASystemPtr = raisimASystemPtr;
        m_graphicsScene = graphicsScene;

        /*** grab some information required to create some resources ***/

        // index of the link|body in the articulated system
        m_localIndx = m_raisimASystemPtr->getBodyIdx( m_name );
        
        // grab vis-collision objects from the articulated system
        auto _visColObjects = m_raisimASystemPtr->getVisColOb();
        for ( size_t i = 0; i < _visColObjects.size(); i++ )
        {
            if ( _visColObjects[i].localIdx != m_localIndx )
                continue;

            // grab the col-shape data
            ShapeData _colShapeData;
            _colShapeData.name      = _visColObjects[i].name;
            _colShapeData.type      = fromRaisimShapeType( _visColObjects[i].shape );
            _colShapeData.localPos  = fromEigenVec3( _visColObjects[i].offset.e() );
            _colShapeData.localRot  = fromEigenMat3( _visColObjects[i].rot.e() );
            _colShapeData.color     = { 0.7, 0.5, 0.3 };
            for ( size_t j = 0; j < _visColObjects[j].visShapeParam.size(); j++ )
            {
                if ( j == 0 )
                    _colShapeData.size.x = _visColObjects[i].visShapeParam[0];
                if ( j == 1 )
                    _colShapeData.size.y = _visColObjects[i].visShapeParam[1];
                if ( j == 2 )
                    _colShapeData.size.z = _visColObjects[i].visShapeParam[2];
            }
            m_colsShapeData.push_back( _colShapeData );

            std::cout << "link-information" << std::endl;
            std::cout << std::to_string( _colShapeData ) << std::endl;

            // compute local-transform of this collision object w.r.t. body object
            m_colsLocalTransforms.push_back( tysoc::TMat4( _colShapeData.localPos, _colShapeData.localRot ) );

            // and create a renderable for this link
            auto _renderable = createRenderableShape( _colShapeData, m_graphicsScene );
            m_graphicsColsObjs.push_back( _renderable );
        }
    }

    void SimLink::update()
    {
        // get the pose information from the articulated system
        m_raisimASystemPtr->getBodyPose( m_localIndx, m_raisimRotation, m_raisimPosition );
        m_worldPos = fromEigenVec3( m_raisimPosition.e() );
        m_worldRot = fromEigenMat3( m_raisimRotation.e() );
        m_worldTransform.setPosition( m_worldPos );
        m_worldTransform.setRotation( m_worldRot );

        // compute world transforms for all collision objects
        for ( size_t i = 0; i < m_colsShapeData.size(); i++ )
        {
            if ( !m_graphicsColsObjs[i] )
                continue;

            auto _localTransform = m_colsLocalTransforms[i];
            auto _worldTransform = m_worldTransform * _localTransform;

            auto _wpos = _worldTransform.getPosition();
            auto _wrot = _worldTransform.getRotation();

            m_graphicsColsObjs[i]->pos = { _wpos.x, _wpos.y, _wpos.z };

            for ( size_t row = 0; row < 3; row++ )
                for ( size_t col = 0; col < 3; col++ )
                    m_graphicsColsObjs[i]->rotation.set( row, col, _wrot.buff[row + 3 * col] );
        }
    }

    /***************************************************************************
    *                                                                          *
    *                         Articulated-System WRAPPER                       *
    *                                                                          *
    ***************************************************************************/

    SimAgent::SimAgent( const std::string& name )
    {
        m_name              = name;
        m_graphicsScene     = NULL;
        m_raisimWorldPtr    = NULL;
        m_raisimASystemPtr  = NULL;
        m_isAlive           = false;
    }

    void SimAgent::update()
    {
        for ( size_t i = 0; i < m_simLinks.size(); i++ )
            m_simLinks[i]->update();
    }

    void SimAgent::build( const std::string& filename,
                          raisim::World* raisimWorldPtr,
                          engine::LScene* graphicsScene )
    {
        // store the references passed to us, for later usage
        m_raisimWorldPtr = raisimWorldPtr;
        m_graphicsScene = graphicsScene;

        // create the ArticulatedSystem using the world and the resource given
        m_raisimASystemPtr = m_raisimWorldPtr->addArticulatedSystem( filename );

        auto _nq = m_raisimASystemPtr->getGeneralizedCoordinateDim();
        auto _nv = m_raisimASystemPtr->getDOF();
        
        std::cout << "nq: " << _nq << std::endl;
        std::cout << "nv: " << _nv << std::endl;

        std::cout << "num-visobj: " << m_raisimASystemPtr->getVisOb().size() << std::endl;
        std::cout << "num-viscolobj: " << m_raisimASystemPtr->getVisColOb().size() << std::endl;

        std::cout << "body-names" << std::endl;
        m_raisimASystemPtr->printOutBodyNamesInOrder();

        auto _bodiesNames = m_raisimASystemPtr->getBodyNames();
        for ( size_t i = 0; i < _bodiesNames.size(); i++ )
        {
            auto _simLinkPtr = new SimLink( _bodiesNames[i],
                                            m_raisimASystemPtr,
                                            m_graphicsScene );

            m_simLinks.push_back( _simLinkPtr );
            m_simLinksMap[ _simLinkPtr->name() ] = _simLinkPtr;
        }

        // make sure this object is marked as alive for other to not delete it
        m_isAlive = true;

        // Eigen::VectorXd gc(2);
        // gc.setZero();
        // gc.segment<2>(0) << 0.5, 1.0;
        // m_raisimASystemPtr->setGeneralizedCoordinate( gc );
    }

    void SimAgent::setPosition( const tysoc::TVec3& position )
    {
        if ( !m_raisimASystemPtr )
            return;

        m_raisimASystemPtr->setBasePos_e( toEigenVec3( position ) );
    }

    void SimAgent::setOrientation( const tysoc::TMat3& rotation )
    {
        if ( !m_raisimASystemPtr )
            return;

        m_raisimASystemPtr->setBaseOrientation_e( toEigenMat3( rotation ) );
    }

    /***************************************************************************
    *                                                                          *
    *                             BASE-APPLICATION                             *
    *                                                                          *
    ***************************************************************************/

    ITestApplication::ITestApplication()
    {
        m_graphicsApp = NULL;
        m_graphicsScene = NULL;

        m_raisimWorldPtr = NULL;

        m_isRunning = false;
        m_isTerminated = false;
    }

    ITestApplication::~ITestApplication()
    {
        for ( size_t i = 0; i < m_simBodies.size(); i++ )
            delete m_simBodies[i];

        m_simBodies.clear();
        m_simBodiesMap.clear();

        // @TODO: Check glfw's master branch, as mujoco's glfw seems be older
        // @TODO: Check why are we linking against glfw3 from mjc libs
    #if defined(__APPLE__) || defined(_WIN32)
        delete m_graphicsApp;
    #endif
        m_graphicsApp = NULL;
        m_graphicsScene = NULL;
    }

    void ITestApplication::init()
    {
        _initScenario();
        _initGraphics();
        _initPhysics();
        _onApplicationStart();

        togglePause();
    }

    void ITestApplication::_initScenario()
    {
        // delegate to virtual method
        _initScenarioInternal();
    }

    void ITestApplication::_initPhysics()
    {
        assert( m_graphicsApp != NULL );
        assert( m_graphicsScene != NULL );

        m_raisimWorldPtr = new raisim::World();
        m_raisimWorldPtr->setTimeStep( 0.002 );

        m_isRunning = true;
        m_isTerminated = false;
    }

    void ITestApplication::_initGraphics()
    {
        m_graphicsApp = engine::LApp::GetInstance();
        m_graphicsScene = engine::LApp::GetInstance()->scene();

        auto _camera = new engine::LFpsCamera( "fps",
                                               { 1.0f, 2.0f, 1.0f },
                                               { -2.0f, -4.0f, -2.0f },
                                               engine::LICamera::UP_Z );

        auto _light = new engine::LLightDirectional( { 0.8, 0.8, 0.8 }, 
                                                     { 0.8, 0.8, 0.8 },
                                                     { 0.3, 0.3, 0.3 }, 
                                                     0, 
                                                     { 0.0, 0.0, -1.0 } );
        _light->setVirtualPosition( { 5.0, 0.0, 5.0 } );

        m_graphicsScene->addCamera( _camera );
        m_graphicsScene->addLight( _light );

        // Initialize UI resources
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;
    #ifdef __APPLE__
        ImGui_ImplOpenGL3_Init( "#version 150" );
    #else
        ImGui_ImplOpenGL3_Init( "#version 130" );
    #endif
        ImGui_ImplGlfw_InitForOpenGL( m_graphicsApp->window()->getGLFWwindow(), false );
        ImGui::StyleColorsDark();

        // disable the current camera movement and allow to use the ui
        m_graphicsScene->getCurrentCamera()->setActiveMode( false );
        m_graphicsApp->window()->enableCursor();
    }

    void ITestApplication::reset()
    {
        // do some specific initialization
        _resetInternal();
    }

    void ITestApplication::step()
    {
        if ( m_isRunning && !m_isTerminated )
        {
            double _tstart = m_raisimWorldPtr->getWorldTime();
            while ( m_raisimWorldPtr->getWorldTime() - _tstart < 1. / 60. )
                m_raisimWorldPtr->integrate();
        }

        // Update all body-wrappers
        for ( size_t i = 0; i < m_simBodies.size(); i++ )
        {
            if ( !m_simBodies[i] )
                continue;

            m_simBodies[i]->update();
        }

        // Update all agent wrappers
        for ( size_t i = 0; i < m_simAgents.size(); i++ )
        {
            if ( !m_simAgents[i] )
                continue;

            m_simAgents[i]->update();
        }

        // do some custom step functionality
        _stepInternal();

        if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_SPACE ) )
        {
            m_graphicsScene->getCurrentCamera()->setActiveMode( false );
            m_graphicsApp->window()->enableCursor();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_ENTER ) )
        {
            m_graphicsScene->getCurrentCamera()->setActiveMode( true );
            m_graphicsApp->window()->disableCursor();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_ESCAPE ) )
        {
            m_isTerminated = true;
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_P ) )
        {
            togglePause();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_Q ) )
        {
            for ( size_t i = 0; i < m_simBodies.size(); i++ )
            {
                if ( !m_simBodies[i] )
                    continue;

                m_simBodies[i]->print();
            }
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_R ) )
        {
            // reset functionality here
            std::cout << "INFO> requested reset of the simulation" << std::endl;
        }

        m_graphicsApp->begin();
        m_graphicsApp->update();

        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 5.0f }, { 0.0f, 0.0f, 1.0f } );
        

        // render the UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        _renderUi();

        ImGui::Render();
        int _ww, _wh;
        glfwGetFramebufferSize( m_graphicsApp->window()->getGLFWwindow(), &_ww, &_wh );
        glViewport( 0, 0, _ww, _wh );
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

        m_graphicsApp->end();
    }

    void ITestApplication::_renderUi()
    {
        // render ui related to agents
        _renderUiAgents();

        // Call some custom render functionality
        _renderUiInternal();
    }

    void ITestApplication::_renderUiAgents()
    {
        // do nothing for now
    }

    void ITestApplication::togglePause()
    {
        m_isRunning = ( m_isRunning ) ? false : true;
    }

    SimBody* ITestApplication::createSingleBody( const ShapeData& shapeData, bool isFree )
    {
        assert( m_raisimWorldPtr != NULL );
        assert( m_graphicsScene != NULL );

        SimBody* _bodyPtr = new SimBody( shapeData.name );

        _bodyPtr->build( m_raisimWorldPtr, m_graphicsScene, shapeData, isFree );
        _bodyPtr->setPosition( shapeData.localPos );
        _bodyPtr->setOrientation( shapeData.localRot );

        m_simBodies.push_back( _bodyPtr );
        m_simBodiesMap[ _bodyPtr->name() ] = _bodyPtr;

        return _bodyPtr;
    }

    SimAgent* ITestApplication::createSingleAgent( const std::string& name,
                                                   const std::string& filename,
                                                   const tysoc::TVec3& position,
                                                   const tysoc::TMat3& rotation )
    {
        assert( m_raisimWorldPtr != NULL );
        assert( m_graphicsScene != NULL );

        SimAgent* _agentPtr = new SimAgent( name );

        _agentPtr->build( filename, m_raisimWorldPtr, m_graphicsScene );
        _agentPtr->setPosition( position );
        _agentPtr->setOrientation( rotation ) ;

        m_simAgents.push_back( _agentPtr );
        m_simAgentsMap[ _agentPtr->name() ] = _agentPtr;

        return _agentPtr;
    }

    SimBody* ITestApplication::getBody( const std::string& name )
    {
        if ( m_simBodiesMap.find( name ) != m_simBodiesMap.end() )
            return m_simBodiesMap[name];

        std::cout << "ERROR> body with name: " << name << " not found" << std::endl;
        return NULL;
    }

}

// some to-string conversion
namespace std
{
    std::string to_string( const raisim::eShapeType& type )
    {
        if ( type == raisim::eShapeType::BOX )
            return "box";

        if ( type == raisim::eShapeType::SPHERE )
            return "sphere";

        if ( type == raisim::eShapeType::CYLINDER )
            return "cylinder";

        if ( type == raisim::eShapeType::CAPSULE )
            return "capsule";

        if ( type == raisim::eShapeType::MESH )
            return "mesh";

        if ( type == raisim::eShapeType::NONE )
            return "none";

        return "undefined";
    }

    std::string to_string( const raisim::eJointType& type )
    {
        if ( type == raisim::eJointType::FREE )
            return "free";

        if ( type == raisim::eJointType::FIXED )
            return "fixed";

        if ( type == raisim::eJointType::REVOLUTE )
            return "revolute";

        if ( type == raisim::eJointType::PRISMATIC )
            return "prismatic";

        if ( type == raisim::eJointType::BALL )
            return "ball";

        if ( type == raisim::eJointType::PLANAR )
            return "planar";

        return "undefined";
    }

    std::string to_string( const raisim::ShapeData& shapeData )
    {
        std::string _res;

        _res += "name       : " + shapeData.name + "\n\r";
        _res += "type       : " + to_string( shapeData.type ) + "\n\r";
        _res += "size       : " + tysoc::TVec3::toString( shapeData.size ) + "\n\r";
        _res += "color      : " + tysoc::TVec3::toString( shapeData.color ) + "\n\r";
        _res += "local-pos  : " + tysoc::TVec3::toString( shapeData.localPos ) + "\n\r";
        // _res += "local-rot  : \n\r" + tysoc::TMat3::toString( shapeData.localRot ) + "\n\r";

        return _res;
    }
}
