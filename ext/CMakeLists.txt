
# Configure dependencies
message( "TYSOCRAISIM> Configuring external dependencies" )

add_subdirectory( cat1 )
add_subdirectory( imgui )
add_subdirectory( pybind11 )

# raisim is added directly in the root cmake file