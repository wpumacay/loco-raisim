
#pragma once

// RAISIM API functionality
#include <raisim/World.hpp>
// Rendering functionality from 'cat1' engine
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// UI functionality (from Dear ImGui)
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// Base functionality (math and a few helpers) from tysoc-core
#include <tysoc_common.h>
#include <memory>
#include <dirent.h>

#define DEFAULT_DENSITY 1000.0f // density of water, same default as in mujoco

namespace raisim
{

    enum class eShapeType
    {
        NONE = 0,
        BOX,
        SPHERE, 
        CYLINDER, 
        CAPSULE, 
        MESH
    };

    eShapeType fromRaisimShapeType( raisim::Shape::Type type );

    enum class eJointType
    {
        FREE = 0,
        REVOLUTE,
        PRISMATIC,
        BALL,
        PLANAR,
        FIXED
    };

    eJointType fromRaisimJointType( raisim::Joint::Type type );

    struct ShapeData
    {
        std::string     name;
        tysoc::TVec3    localPos;   // relative position w.r.t. parent body
        tysoc::TMat3    localRot;   // relative rotation w.r.t. parent body
        eShapeType      type;       // type of collision shape of this body
        tysoc::TVec3    size;       // size of the collision shape of this body
        tysoc::TVec3    color;      // color (rgb) of the renderable associated with this shape
    };

    struct JointData
    {
        std::string     name;
        tysoc::TVec3    pos;                // relative position w.r.t. parent body
        eJointType      type;               // type of joint to be constructed
        tysoc::TVec3    axis;               // axis of joint to be constructed
        tysoc::TVec2    limits;             // motion range (lo==hi: fixed, lo>hi: continuous, lo<hi: limited)
    };

    std::vector< std::string > collectAvailableModels( const std::string& folderpath );

    Eigen::Vector3d toEigenVec3( const tysoc::TVec3& vec );
    Eigen::Matrix3d toEigenMat3( const tysoc::TMat3& mat );

    tysoc::TVec3 fromEigenVec3( const Eigen::Vector3d& vec );
    tysoc::TMat3 fromEigenMat3( const Eigen::Matrix3d& mat );

    /**
    *   Wrapper for a single body with geometries as children
    */
    class SimBody
    {
        protected :

        std::string     m_name;
        tysoc::TVec3    m_worldPos;
        tysoc::TMat3    m_worldRot;
        tysoc::TMat4    m_worldTransform;

        engine::LScene*         m_graphicsScene;
        engine::LIRenderable*   m_graphicsObj;

        raisim::World*              m_raisimWorldPtr;
        raisim::SingleBodyObject*   m_raisimBodyPtr;

        bool m_isAlive;

        public :

        /* Constructs the body wrapper given the name of the body */
        SimBody( const std::string& bodyName );

        /* Frees|unlinks all related resources of the currently wrapped body */
        ~SimBody() {}

        void build( raisim::World* raisimWorldPtr,
                    engine::LScene* graphicsScene,
                    const ShapeData& shapeData,
                    bool isFree );

        /* Updates all internal information by quering the backend */
        void update();

        /* Returns the unique name of the wrapped body */
        std::string name() { return m_name; }

        /* Returns the graphics resource of this wrapper */
        engine::LIRenderable* graphics() { return m_graphicsObj; }

        /* Resets the body to some configuration */
        void reset() {}

        /* Prints some debug information */
        void print() {}

        /* Sets the position of this body */
        void setPosition( const tysoc::TVec3& position );

        /* Sets the orientation of this body */
        void setOrientation( const tysoc::TMat3& orientation );

        /* Sets the alive-mode of this body */
        void setAlive( bool mode ) { m_isAlive = mode; }

        /* Returns whether or not this body is still alive */
        bool alive() { return m_isAlive; }

        /* Returns the world-position of the body */
        tysoc::TVec3 position() { return m_worldPos; }

        /* Returns the world-orientation of the body */
        tysoc::TMat3 orientation() { return m_worldRot; }

        /* Returns the world transform of the body */
        tysoc::TMat4 worldTransform() { return m_worldTransform; }
    };

    class SimLink
    {
        private :

        int             m_localIndx;
        std::string     m_name;

        tysoc::TVec3    m_worldPos;
        tysoc::TMat3    m_worldRot;
        tysoc::TMat4    m_worldTransform;

        raisim::Vec<3>      m_raisimPosition;
        raisim::Mat<3, 3>   m_raisimRotation;

        engine::LScene*             m_graphicsScene;
        raisim::ArticulatedSystem*  m_raisimASystemPtr;

        std::vector< ShapeData >                m_colsShapeData;
        std::vector< tysoc::TMat4 >             m_colsLocalTransforms;
        std::vector< engine::LIRenderable* >    m_graphicsColsObjs;

        public :

        SimLink( const std::string& name,
                 raisim::ArticulatedSystem* raisimASystemPtr,
                 engine::LScene* graphicsScene );

        ~SimLink() {}

        void reset() {}

        void update();

        std::string name() { return m_name; }
    };

    /**
    *   A wrapper of an agent|articulated-system parsed from 
    *   a urdf file. This class just wraps the functionality
    *   and provides helper accessors to the functionality of
    *   the backend (just to play around).
    */
    class SimAgent
    {
        private :

        std::string     m_name;
        tysoc::TVec3    m_worldPos;
        tysoc::TMat3    m_worldRot;
        tysoc::TMat4    m_worldTransform;

        engine::LScene*             m_graphicsScene;
        raisim::World*              m_raisimWorldPtr;
        raisim::ArticulatedSystem*  m_raisimASystemPtr;

        std::vector< SimLink* >             m_simLinks;
        std::map< std::string, SimLink* >   m_simLinksMap;

        bool m_isAlive;

        public :

        SimAgent( const std::string& name );

        SimAgent() {}

        void build( const std::string& filename,
                    raisim::World* raisimWorldPtr,
                    engine::LScene* graphicsScene );

        void update();

        void reset() {}

        void print() {}

        std::string name() { return m_name; }

        void setPosition( const tysoc::TVec3& position );

        void setOrientation( const tysoc::TMat3& rotation );

        void setAlive( bool mode ) { m_isAlive = mode; }

        bool alive() { return m_isAlive; }

        tysoc::TVec3 position() { return m_worldPos; }

        tysoc::TMat3 orientation() { return m_worldRot; }

        tysoc::TMat4 worldTransform() { return m_worldTransform; }

        std::vector< SimLink* > links() { return m_simLinks; }
    };

    class ITestApplication
    {

        protected :

        raisim::World* m_raisimWorldPtr;

        engine::LApp*   m_graphicsApp;
        engine::LScene* m_graphicsScene;

        std::vector< SimBody* >             m_simBodies;
        std::map< std::string, SimBody* >   m_simBodiesMap;

        std::vector< SimAgent* >            m_simAgents;
        std::map< std::string, SimAgent* >  m_simAgentsMap;

        bool m_isRunning;
        bool m_isTerminated;

        void _initScenario();
        void _initPhysics(); 
        void _initGraphics();

        void _renderUi();
        void _renderUiAgents();

        // User should override this and create what he wants
        virtual void _initScenarioInternal() = 0;
        // Special functionality used after taking a step
        virtual void _stepInternal() {}
        // Special functionality used before calling reset
        virtual void _resetInternal() {}
        // UI functionality
        virtual void _renderUiInternal() {}
        // Entry-point for when the application is fully created
        virtual void _onApplicationStart() {}

        public :

        ITestApplication();
        ~ITestApplication();

        void init();
        void reset();
        void step();
        void togglePause();

        SimBody* createSingleBody( const ShapeData& shapeData, bool isFree );
        SimAgent* createSingleAgent( const std::string& name,
                                     const std::string& filename,
                                     const tysoc::TVec3& position,
                                     const tysoc::TMat3& rotation );

        bool isTerminated() { return m_isTerminated; }

        /* Returns a body-wrapper of a body with a specific name in the simulation */
        SimBody* getBody( const std::string& name );
    };

}

// some to-string conversion
namespace std
{
    std::string to_string( const raisim::eShapeType& type );
    std::string to_string( const raisim::eJointType& type );
    std::string to_string( const raisim::ShapeData& shapeData );
}