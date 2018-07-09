project(ubitrack_device_camera_avfoundation)

cmake_minimum_required(VERSION 3.4.0)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)


if(APPLE)
	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} "-framework AVFoundation")
	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} "-framework CoreMedia")
	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} "-framework QuartzCore")
	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} "-framework AppKit")

	add_subdirectory(src/AVFoundationCapture)

	ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
	ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
ENDIF(APPLE)