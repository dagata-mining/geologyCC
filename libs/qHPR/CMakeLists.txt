project( QHPR_LIB )

add_library( ${PROJECT_NAME} SHARED )


	
add_subdirectory( extern/qhull EXCLUDE_FROM_ALL )
add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

target_compile_definitions( ${PROJECT_NAME} PUBLIC HPR_BUILD )
	
target_link_libraries( ${PROJECT_NAME} qhull_lib )

InstallSharedLibrary( TARGET ${PROJECT_NAME} )