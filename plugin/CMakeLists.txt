include_directories( ${OCC_INCLUDE_DIR} )
add_library( s3d_plugin_oce MODULE oce.cpp loadmodel.cpp )
#link_directories( ${OCC_LIBRARY_DIR} )
target_link_libraries( s3d_plugin_oce kicad_3dsg ${OCC_LIBRARIES} ${OCC_OCAF_LIBRARIES} ${wxWidgets_LIBRARIES} )
message( WARNING ${OCC_LIBRARY_DIR} )

if( APPLE )
    # puts library into the main kicad.app bundle in build tree
    set_target_properties( s3d_plugin_oce PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${OSX_BUNDLE_BUILD_PLUGIN_DIR}/3d"
						LINK_FLAGS -L${OCC_LIBRARY_DIR}
            )
endif()
set_target_properties( s3d_plugin_oce PROPERTIES LINK_FLAGS
	-L${OCC_LIBRARY_DIR} )

install( TARGETS
        s3d_plugin_oce
        DESTINATION ${KICAD_USER_PLUGIN}/3d
        COMPONENT binary
        )
