
#ifndef POINT_CLOUD_RASTERIZER_
#define POINT_CLOUD_RASTERIZER_

#include <pcl/point_types.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>

#include <gdal/gdal_priv.h>
#include <limits>

#include <pcl/kdtree/kdtree_flann.h>

template <typename PointType>
class PointCloudRaster
{
public:
    typedef typename pcl::PointCloud<PointType> PC;
    typedef typename pcl::PointCloud<PointType>::Ptr PCP;
    typedef typename pcl::KdTreeFLANN<PointType> KD;
    typedef typename pcl::KdTreeFLANN<PointType>::Ptr KDP;

    // Take an input PCL PointCloud Ptr, output a 2D raster matrix containing a list of all the points in each cell
    PointCloudRaster(PCP cloud, float pixel_width, float pixel_height, Eigen::Vector2f origin=Eigen::Vector2f::Zero(), bool debugging=false);
    // Verify that a raster has actually been populated
    //   For a gridded set of points
    bool checkRasterInitialization();
    //   For a real raster 
    template <typename DataType>
    bool checkRasterInitialization(std::vector<std::vector<DataType> >& raster);

    // ***** Statistics *****
    void generateMaxRaster(std::string field_name, std::vector<std::vector<float> >& raster_out, float default_value=-9999);
    void generateMinRaster(std::string field_name, std::vector<std::vector<float> >& raster_out, float default_value=-9999);
    void generateMedianRaster(std::string field_name, std::vector<std::vector<float> >& raster_out, float default_value=-9999);
    void generateDensityRaster(std::vector<std::vector<float> >& raster_out);

    template <typename DataType> 
    void outputTIF(std::vector<std::vector<DataType> > image, std::string filename);

private:
    PCP cloud_;
    KDP tree_;

    // 2D output raster containing list of point cloud indices within each pixel
    std::vector<std::vector<std::vector<int> > > index_raster_;
    // Mapping from cloud index to raster indices
    std::vector<std::pair<int, int> > index_map_;

    float pixel_width_; 
    float pixel_height_;
    Eigen::Vector2f origin_;
    float height;
    float width;

    float getFieldValue(PointType point, std::string field_name);
};

#endif //POINT_CLOUD_RASTERIZER_