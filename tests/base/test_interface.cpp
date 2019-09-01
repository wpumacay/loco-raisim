
#include "test_interface.h"


namespace raisim
{

    /***************************************************************************
    *                                                                          *
    *                               UTILITIES                                  *
    *                                                                          *
    ***************************************************************************/

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
}