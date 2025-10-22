cmake_minimum_required(VERSION 3.16)

project(mask2cluster LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(M2C_ENABLE_BUILD "Enable building mask2cluster targets" OFF)
option(M2C_WITH_PDAL "Enable LAS reading via PDAL" ON)

# TODO: enable build targets once interfaces and sources are implemented.
# TODO: add find_package(PCL REQUIRED)
# TODO: add find_package(Eigen3 REQUIRED)

if(M2C_WITH_PDAL)
	find_package(PDAL QUIET)
	if(NOT PDAL_FOUND)
		message(WARNING "PDAL not found; LAS ingestion will remain unavailable until the dependency is installed.")
	endif()
endif()

# NOTE: No executables or libraries are declared in this scaffolding milestone.
