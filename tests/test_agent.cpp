
#include <test_interface.h>
#include <random>

class AppExample : public raisim::ITestApplication
{
    private :

    raisim::SimBody* m_samplePlanePtr;
    raisim::SimAgent* m_sampleAgentPtr;

    protected :

    void _initScenarioInternal() override;
    void _onApplicationStart() override;
    void _stepInternal() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : raisim::ITestApplication()
{
    // nothing here
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenarioInternal()
{
    // nothing to do here
}

void AppExample::_onApplicationStart()
{
    std::cout << "INFO> Application has started" << std::endl;

    raisim::ShapeData _planeShapeData;
    _planeShapeData.name = "gplane";
    _planeShapeData.type = raisim::eShapeType::BOX;
    _planeShapeData.size = { 10.0, 10.0, 0.2 };
    _planeShapeData.color = { 0.2, 0.4, 0.6 };
    _planeShapeData.localPos = { 0.0, 0.0, -0.1 };

    m_samplePlanePtr = createSingleBody( _planeShapeData, false );

    m_sampleAgentPtr = createSingleAgent( "agent0",
                            "/home/gregor/Documents/repos/tysoc_raisim_workspace/tysoc_raisim/core/res/xml/tests/raisim/double_pendulum.urdf",
                            tysoc::TVec3( 0.0, 0.0, 3.0 ),
                            tysoc::TMat3::fromEuler( { 0.0, 0.0, 0.0 } ) );
}

void AppExample::_stepInternal()
{
    
}

int main()
{

    auto _app = new AppExample();

    _app->init();
    _app->reset();

    while ( !_app->isTerminated() )
        _app->step();

    delete _app;

    return 0;
}