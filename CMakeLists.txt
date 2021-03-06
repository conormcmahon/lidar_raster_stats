cmake_minimum_required(VERSION 2.8.3)

project(lidar_raster_stats)
include_directories(include "/usr/include")

# Point Cloud Library (PCL)
find_package(PCL 1.09 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Computational Geometry Algorithms Library (CGAL)
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL QUIET COMPONENTS)
if(NOT CGAL_FOUND)
  message(STATUS "This project requires CGAL and will not be compiled.")
  return()
endif()

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# GDAL
find_package(GDAL REQUIRED)

add_library(base_raster_classes src/base_raster_classes.cpp)
target_link_libraries(base_raster_classes ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})

add_executable(raster_stats_tester src/raster_stats_tester.cpp)
target_link_libraries(raster_stats_tester base_raster_classes ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES} ${PDAL_LIBRARIES})

add_executable(dem_raster_from_tin src/dem_raster_from_tin.cpp)
target_link_libraries(dem_raster_from_tin base_raster_classes ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES} ${PDAL_LIBRARIES})

add_executable(terrain_and_hydrology_from_tin src/terrain_and_hydrology_from_tin.cpp)
target_link_libraries(terrain_and_hydrology_from_tin base_raster_classes ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES} ${PDAL_LIBRARIES})


# Install 
install(TARGETS base_raster_classes
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
