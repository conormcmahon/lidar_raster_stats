
#include <lidar_raster_stats/point_cloud_raster.hpp>
#include <lidar_raster_stats/TIN_raster.hpp>
#include <lidar_raster_stats/TIN_hydrologic_raster.hpp>

#include <dirt_or_leaf/point_2d_ground.h>
#include <dirt_or_leaf/point_veg.h>
#include <dirt_or_leaf/point_las.h>
#include <riparian_hydrology_pcl/point_channel.h>

// This file just declares and forces compilation for some template targets from the class template
// Point Type source code at https://github.com/conormcmahon/dirt_or_leaf/include/...
// ------------------------------------------------------------------------------------------------
//   Point2DGround - 2.5D data structure with:
//      curvature
//      height_diff_avg
//      norm_diff_avg
//      slope
//      aspect
//      height_over_stream
//      dist_from_stream
template class PointCloudRaster<pcl::Point2DGround>;
template class TINRaster<pcl::Point2DGround>;              // this class can only be build for points with SLOPE, ASPECT, HEIGHT_OVER_STREAM, DIST_FROM_STREAM
template class TINHydrologicRaster<pcl::Point2DGround, pcl::PointChannel>;     // as above, but the second point type MUST be a 2.5 D point type (search in XY space only, not Z - see pcl::DefaultPointRepresentation)
// ------------------------------------------------------------------------------------------------
//   PointVeg - 3D data structure with:
//      intensity
//      classification
//      height
//      returnnumber
//      numberofreturns
//      roughness
//      index
template class PointCloudRaster<pcl::PointVeg>;
// ------------------------------------------------------------------------------------------------
//   PointLAS - 3D data structure with:
//      intensity
//      classification
//      returnnumber
//      numberofreturns
template class PointCloudRaster<pcl::PointLAS>;