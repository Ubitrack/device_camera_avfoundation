set(the_description "The OSX AVFoundation Capture Component")
ut_add_component(device_camera_avfoundation DEPS utcore utdataflow utvision )

if(APPLE AND CMAKE_COMPILER_IS_CLANGCXX)
    set(HAVE_AVFOUNDATION YES)
    list(APPEND AVFOUNDATION_LIBRARIES "-framework AVFoundation" "-framework CoreMedia" "-framework QuartzCore" "-framework AppKit")

	ut_component_include_directories("src/AVFoundationCapture" ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})
	ut_glob_component_sources(SOURCES "src/AVFoundationCapture/AVFoundationCapture.mm")
	ut_create_single_component(${AVFOUNDATION_LIBRARIES})
	ut_install_utql_patterns()
ENDIF()
