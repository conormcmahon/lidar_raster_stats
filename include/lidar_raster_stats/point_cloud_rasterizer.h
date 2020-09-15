
#ifndef POINT_CLOUD_RASTERIZER_
#define POINT_CLOUD_RASTERIZER_

#include <pcl/point_types.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>

namespace point_cloud_rasterizer
{
    // 2D output raster containing list of cloud indices within each pixel
    typedef typename std::vector<std::vector<std::vector<int> > > index_raster;
    // Mapping from cloud index to raster indices
    typedef typename std::vector<std::pair<int, int> > index_map;


    // Take an input PCL PointCloud Ptr, output a 2D raster matrix containing a list of all the points in each cell
    template <typename CloudType>
    void rasterize(CloudType cloud, index_raster& raster_indices, index_map& map, float pixel_width, float pixel_height, Eigen::Vector2f origin=Eigen::Vector2f::Zero(), bool debugging=false);
    // Print a simple rasterized point cloud showing the point density in each output pixel within the Intensity channel 
    int printDensityCloud(index_raster& raster, std::string filename);
}

#endif //POINT_CLOUD_RASTERIZER_