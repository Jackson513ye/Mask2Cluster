cmake_minimum_required(VERSION 3.16)

project(mask2cluster LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(M2C_ENABLE_BUILD "Enable building the mask2cluster CLI" OFF)
option(M2C_WITH_PDAL "Enable LAS reading via PDAL" ON)
option(M2C_BUILD_TOOLS "Build development utilities" OFF)

# TODO: enable build targets once interfaces and sources are implemented.
# TODO: add find_package(PCL REQUIRED)
# TODO: add find_package(Eigen3 REQUIRED)

if(M2C_WITH_PDAL)
	find_package(PDAL QUIET)
	if(NOT PDAL_FOUND)
		message(WARNING "PDAL not found; LAS ingestion will remain unavailable until the dependency is installed.")
	endif()
endif()

if(M2C_ENABLE_BUILD)
	find_package(PCL REQUIRED COMPONENTS io filters search kdtree)
	find_package(Eigen3 REQUIRED)

	add_executable(mask2cluster
		apps/mask2cluster.cpp
		src/io_las.cpp
		src/io_pose.cpp
		src/kdtree.cpp
		src/pipeline.cpp
		src/validator.cpp
	)

	target_compile_features(mask2cluster PRIVATE cxx_std_17)
	target_include_directories(mask2cluster
		PRIVATE
			${PCL_INCLUDE_DIRS}
			${EIGEN3_INCLUDE_DIRS}
			${PDAL_INCLUDE_DIRS}
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_CURRENT_SOURCE_DIR}/third_party
	)

	target_link_libraries(mask2cluster
		PRIVATE
			  ${PCL_LIBRARIES}
			  Eigen3::Eigen
	)

	if(PDAL_FOUND)
		target_link_libraries(mask2cluster PRIVATE ${PDAL_LIBRARIES})
		target_compile_definitions(mask2cluster PRIVATE M2C_HAS_PDAL)
	endif()

	if(PCL_DEFINITIONS)
		target_compile_definitions(mask2cluster PRIVATE ${PCL_DEFINITIONS})
	endif()

	target_compile_definitions(mask2cluster PRIVATE M2C_WITH_PDAL=$<BOOL:${M2C_WITH_PDAL}>)
endif()

if(M2C_BUILD_TOOLS)
	find_package(PCL REQUIRED COMPONENTS io search kdtree)
	find_package(Eigen3 REQUIRED)

	add_executable(loader_probe
		apps/loader_probe.cpp
		src/io_las.cpp
		src/io_pose.cpp
	)

	add_executable(kd_probe
		apps/kd_probe.cpp
		src/io_las.cpp
		src/kdtree.cpp
	)

	target_compile_features(loader_probe PRIVATE cxx_std_17)
	target_include_directories(loader_probe
		PRIVATE
			${PCL_INCLUDE_DIRS}
			${EIGEN3_INCLUDE_DIRS}
			${PDAL_INCLUDE_DIRS}
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_CURRENT_SOURCE_DIR}/third_party
	)

	target_compile_features(kd_probe PRIVATE cxx_std_17)
	target_include_directories(kd_probe
		PRIVATE
			${PCL_INCLUDE_DIRS}
			${EIGEN3_INCLUDE_DIRS}
			${PDAL_INCLUDE_DIRS}
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_CURRENT_SOURCE_DIR}/third_party
	)

	target_link_libraries(loader_probe PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
	target_link_libraries(kd_probe PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)

	if(PDAL_FOUND)
		target_link_libraries(loader_probe PRIVATE ${PDAL_LIBRARIES})
		target_link_libraries(kd_probe PRIVATE ${PDAL_LIBRARIES})
		target_compile_definitions(loader_probe PRIVATE M2C_HAS_PDAL)
		target_compile_definitions(kd_probe PRIVATE M2C_HAS_PDAL)
	endif()

	if(PCL_DEFINITIONS)
		target_compile_definitions(loader_probe PRIVATE ${PCL_DEFINITIONS})
		target_compile_definitions(kd_probe PRIVATE ${PCL_DEFINITIONS})
	endif()

	target_compile_definitions(loader_probe PRIVATE M2C_WITH_PDAL=$<BOOL:${M2C_WITH_PDAL}>)
	target_compile_definitions(kd_probe PRIVATE M2C_WITH_PDAL=$<BOOL:${M2C_WITH_PDAL}>)
endif()
