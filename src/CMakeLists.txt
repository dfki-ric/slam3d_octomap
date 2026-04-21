add_library(octomap SHARED
	OctoMap.cpp
)

target_include_directories(octomap
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(octomap
	PUBLIC
		slam3d::sensor-pcl
		octomap::octomap
)

# Install header files
install(DIRECTORY ./
	DESTINATION src
	FILES_MATCHING PATTERN "*.hpp"
)

# Install the binaries
install(TARGETS octomap EXPORT slam3d_octomap-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_octomap.pc.in slam3d_octomap.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/src/slam3d_octomap.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(octomap PROPERTIES
	OUTPUT_NAME slam3d_octomap
)
