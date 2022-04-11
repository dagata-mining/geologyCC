project( QCC_IO_LIB )

add_library( ${PROJECT_NAME} SHARED )

include( cmake/GDALSupport.cmake )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

option( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" ON )
if( ${OPTION_USE_DXF_LIB} )
	set( DXF_LIB_DIR "extern/dxflib" CACHE PATH "DXF lib path" )
	add_subdirectory( "${DXF_LIB_DIR}" )
	target_link_libraries( ${PROJECT_NAME} dxflib  )

	#set( YAML_LIB_DIR "extern/yaml-cpp" CACHE PATH "Yaml lib path" )
	#add_subdirectory( "${YAML_LIB_DIR}" )
	#target_link_libraries( ${PROJECT_NAME} yaml-cpp  )
	
	#find_package(nlohmann_json CONFIG REQUIRED)
	#include_directories(${YAML_CPP_INCLUDE_DIRS})
	
	#target_link_libraries( ${PROJECT_NAME} ${YAML_CPP_LIBRARIES} "C:/Program Files/YAML_CPP/lib/libyaml-cppmd.lib" "C:/Program Files/YAML_CPP/lib/libyaml-cppmt.lib")
	
	#target_link_libraries( ${PROJECT_NAME} nlohmann_json)
	#target_link_libraries( ${PROJECT_NAME} "C:/Program Files/YAML_CPP/lib/yaml-cpp.lib"  )
	#target_include_directories( ${PROJECT_NAME} PUBLIC "C:/Program Files/YAML_CPP/include/yaml-cpp"  )

	
endif()

option( OPTION_USE_SHAPE_LIB "Build with ShapeLib (SHP files support)" ON )
if( ${OPTION_USE_SHAPE_LIB} )
    set( SHAPELIB_LIB_DIR "extern/shapelib" CACHE PATH "shapelib lib path" )
	add_subdirectory( "${SHAPELIB_LIB_DIR}" )
	target_link_libraries( ${PROJECT_NAME} shapelib )	
endif()



target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
)

if( OPTION_USE_GDAL )
	target_link_GDAL( ${PROJECT_NAME} )
endif()

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} )

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_IO_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
	CXX_VISIBILITY_PRESET hidden
)

if ( BUILD_TESTING )
	add_subdirectory( test ) 
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )


