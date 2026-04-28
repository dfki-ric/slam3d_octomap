add_library(slam3d_octomap SHARED
	OctoMap.cpp
)

target_include_directories(slam3d_octomap
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(slam3d_octomap
	PUBLIC
		slam3d::sensor-pcl
		octomap
)

# Install header files
install(DIRECTORY ./
	DESTINATION include/slam3d_octomap
	FILES_MATCHING PATTERN "*.hpp"
)

# Install the binaries
install(TARGETS slam3d_octomap EXPORT slam3d_octomap-targets
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
