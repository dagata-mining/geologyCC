option( PLUGIN_IO_DXFWR "Install DXFWR" ON )

if( PLUGIN_IO_DXFWR )
	project( DXFWR )

	#AddPlugin( NAME ${PROJECT_NAME} TYPE io )
	#add_library( ${PROJECT_NAME} STATIC )

	add_subdirectory( include )
	add_subdirectory( src )
	#find_package( libdxfrw REQUIRED )
	target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${libdxfrw_INCLUDE_DIRS}
		
	)

	target_link_directories( ${PROJECT_NAME}
		PUBLIC
		    ${libdxfrw_LIBRARY_DIRS}
	)
	target_link_libraries( ${PROJECT_NAME}
	CCPluginAPI
	QCC_GL_LIB
	QCC_DB_LIB
	QCC_IO_LIB
	${libdxfrw_LIBRARIES}
	)
	
	
endif()
