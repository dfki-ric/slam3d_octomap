cmake_minimum_required(VERSION 3.1)

project(slam3d_octomap VERSION "0.0.0")

include(slam3d_octomap-dependencies.cmake)

add_subdirectory(src)

# Export the library interface
install(EXPORT slam3d_octomap-targets
	NAMESPACE slam3d_octomap::
	DESTINATION lib/cmake/slam3d_octomap
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("slam3d_octomap-config-version.cmake"
	VERSION ${SLAM3D_OCTOMAP_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
		slam3d_octomap-config.cmake
		slam3d_octomap-dependencies.cmake
		${PROJECT_BINARY_DIR}/slam3d_octomap-config-version.cmake
	DESTINATION
		lib/cmake/slam3d_octomap
)
