
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS " -g ${CMAKE_CXX_FLAGS}" )

# Include CMake helper modules
include(GNUInstallDirs)

if ( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    message( "TYSOC_RAISIM> Sending generated libs to OWN build directory" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs" )
else()
    message( "TYSOC_RAISIM> Sending generated libs to EXTERNAL build directory" )
endif()

message( "TYSOC_RAISIM> LIBRARY_OUT_DIR: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

set( TYSOC_RAISIM_IS_MASTER_PROJECT OFF )
if( CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( "TYSOC_RAISIM> Configuring as MASTER project" )
    set( TYSOC_RAISIM_IS_MASTER_PROJECT ON )
else()
    message( "TYSOC_RAISIM> Configuring as CHILD project" )
endif()

# Add core targets only if master project. If not, parent project must configure it
if ( TYSOC_RAISIM_IS_MASTER_PROJECT )
    # Configure dependencies for tysoc
    add_subdirectory( ext )
    # Configure core functionality
    add_subdirectory( core )
endif()

# Configure TYSOC_RAISIM project ************************************************

# Set the lbirary paths as props to allow usage from other projects
set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_RAISIM "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsRaisim.so" )

# get properties defined in core library
get_property( VAR_TYSOC_PATH_RESOURCES              GLOBAL PROPERTY PROP_TYSOC_PATH_RESOURCES )
get_property( VAR_TYSOC_PATH_WORKING_DIR            GLOBAL PROPERTY PROP_TYSOC_PATH_WORKING_DIR )
get_property( VAR_TYSOC_PATH_MJCF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_MJCF_TEMPLATES )
get_property( VAR_TYSOC_PATH_URDF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_URDF_TEMPLATES )
get_property( VAR_TYSOC_PATH_RLSIM_TEMPLATES        GLOBAL PROPERTY PROP_TYSOC_PATH_RLSIM_TEMPLATES )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ      GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ )
get_property( VAR_TYSOC_BACKEND_PHYSICS_RAISIM      GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_RAISIM )

# repeat definitions from core library, as they are not propagated up
add_definitions( -DTYSOC_PATH_RESOURCES="${VAR_TYSOC_PATH_RESOURCES}" )
add_definitions( -DTYSOC_PATH_WORKING_DIR="${VAR_TYSOC_PATH_WORKING_DIR}" )
add_definitions( -DTYSOC_PATH_MJCF_TEMPLATES="${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_URDF_TEMPLATES="${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_RLSIM_TEMPLATES="${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_GLVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
add_definitions( -DTYSOC_BACKEND_PHYSICS_RAISIM="${VAR_TYSOC_BACKEND_PHYSICS_RAISIM}" )

# test all properties
message( "PROP_TYSOC_PATH_RESOURCES:            ${VAR_TYSOC_PATH_RESOURCES}" )
message( "PROP_TYSOC_PATH_WORKING_DIR:          ${VAR_TYSOC_PATH_WORKING_DIR}" )
message( "PROP_TYSOC_PATH_MJCF_TEMPLATES:       ${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_URDF_TEMPLATES:       ${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_RLSIM_TEMPLATES:      ${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ:    ${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
message( "PROP_TYSOC_BACKEND_PHYSICS_RAISIM:    ${VAR_TYSOC_BACKEND_PHYSICS_RAISIM}" )

# grab all the sources for compilation
file( GLOB_RECURSE TYSOC_RAISIM_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
message( "TYSOC_RAISIM_SOURCES: ${TYSOC_RAISIM_SOURCES}" )

# add this project headers to includepath
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )

# include_dirs for tysoc-core dependency: either case (master or non-master project) ... 
# we should find it at root level
include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )

find_package( assimp REQUIRED )
find_package( Eigen3 REQUIRED HINTS ${Eigen3_HINT} )
include_directories( ${EIGEN3_INCLUDE_DIR} )

# add raisim includes, which can be found in either this repo's ext ...
# folder, or in the master's repo ext folder
include_directories( "${CMAKE_SOURCE_DIR}/ext/raisim/include" )


set( CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/ext/raisim" )
find_package( raisim CONFIG REQUIRED )

# Configure the resulting library
add_library( tysocPhysicsRaisim SHARED
             ${TYSOC_RAISIM_SOURCES} )
target_link_libraries( tysocPhysicsRaisim
                       tysocCore
                       assimp
                       raisim::raisim
                       tcmalloc )

# ******************************************************************************

# Only generate samples if not using as dependency
if ( TYSOC_RAISIM_IS_MASTER_PROJECT )
    ## # Configure examples
    ## add_subdirectory( examples )
    ## # Configure tests
    ## add_subdirectory( tests )
endif()