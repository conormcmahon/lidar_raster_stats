cmake_minimum_required(VERSION 2.8.3)

project(pcl_test)

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


add_library(point_cloud_rasterizer src/point_cloud_rasterizer.cpp)
target_link_libraries(point_cloud_rasterizer ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})

add_executable(raster_stats_tester src/raster_stats_tester.cpp)
target_link_libraries(raster_stats_tester ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})
