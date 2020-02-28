message( "LOCO::RAISIM::tests::cpp >>> Configuring C/C++ loco-raisim tests" )

include_directories( "${LOCO_RAISIM_INCLUDE_DIRS}" )

function( FcnBuildRaisimTest pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
endfunction()

FILE( GLOB TestRaisimSources *.cpp )

foreach( testRaisimFile ${TestRaisimSources} )
    string( REPLACE ".cpp" "" executableLongName ${testRaisimFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildRaisimTest( ${testRaisimFile} ${execName} )
endforeach( testRaisimFile )