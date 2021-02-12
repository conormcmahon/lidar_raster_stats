
#ifndef POINT_CLOUD_RASTER_
#define POINT_CLOUD_RASTER_

#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>

#include <dirt_or_leaf/runtime_field_selection.hpp>

#include <limits>

struct HistogramOptions
{
    int num_bins;
    
    // Options for how histogram range is determined
    bool min_max_specified; // user specifies a global min and max value for the histogram range
    bool scaled_by_pixel; // for each pixel, make the histogram run from the min to the max value available (save these values in first two bands)
    // if neither of the above is true, generate a single histogram range based on the global cloud min/max

    float min_value;
    float max_value;
};

template <typename PointType>
class PointCloudRaster
{
public:
    typedef typename pcl::PointCloud<PointType> PC;
    typedef typename pcl::PointCloud<PointType>::Ptr PCP;

    typedef typename std::vector<std::vector<float> > float_raster;
    typedef typename std::vector<std::vector<std::vector<float> > > histogram_raster;

    // Constructor
    PointCloudRaster(float pixel_width, float pixel_height, Eigen::Vector2f origin_offset=Eigen::Vector2f::Zero());
    // Destructor (make sure to release all GDAL-associated memory)
    ~PointCloudRaster();
    // Load Data, optionally reproject, and build raster
    //    Load cloud from existing PCL object in memory
    void buildRasterStructure(PCP cloud, int EPSG, int EPSG_reproj=0, float scale_factor=1);
    //    Load cloud from .PCD file 
    void buildRasterStructure(std::string filename, int EPSG, int EPSG_reproj=0, float scale_factor=1);
    // Verify that a raster has actually been populated
    //   For a gridded set of points
    bool checkRasterInitialization();
    //   For a real raster 
    template <typename DataType>
    bool checkRasterInitialization(std::vector<std::vector<DataType> >const &raster);

    // Return some Raster Structural Information
    Eigen::Vector2f getOrigin();
    Eigen::Vector2i getImageSize();
    Eigen::Vector2f getPixelSize();

    // ***** Statistics *****
    void generateMinRaster(std::string field_name, float_raster &raster_out, float default_value=-9999, float scale_factor=1);
    void generateMaxRaster(std::string field_name, float_raster &raster_out, float default_value=-9999, float scale_factor=1);
    void generateMedianRaster(std::string field_name, float_raster &raster_out, float default_value=-9999, float scale_factor=1);
    void generatePercentileRaster(std::string field_name, float_raster &raster_out, float percentile, float default_value=-9999, float scale_factor=1);
    void generateDensityRaster(float_raster &raster_out);
    void generateHistogram(std::string field_name, histogram_raster &raster_out, HistogramOptions opt, float default_value=-9999, float scale_factor=1);

    void outputTIF(float_raster const &image, std::string filename, GDALDriver *driver);
    void outputTIFMultiband(histogram_raster const &image, std::string filename, GDALDriver *driver);

protected:
    // Point Data 
    PCP cloud_;

    // 2D output raster containing list of point cloud indices within each pixel
    std::vector<std::vector<std::vector<int> > > index_raster_;
    // Mapping from cloud index to raster indices
    std::vector<std::pair<int, int> > index_map_;

    // Dimensions of one pixel in CRS units
    float pixel_width_; 
    float pixel_height_;
    // origin_offset is user-specified parameter determining offset of infinite raster grid from {0,0}
    Eigen::Vector2f origin_offset_;
    // origin is class-calculated origin of output raster, translated an integer multiple of pixel_width and pixel_height from {0,0} to include all data points
    Eigen::Vector2f origin_;
    // Height of output image in pixels
    float height_;
    // Width of output image in pixels
    float width_;
    // EPSG code for coordinate reference system of input data
    int EPSG_;

    // Raster Data Manipulation
    //   Build Raster Structure
    void buildRaster();
    //   Reproject between two EPSG CRSs 
    void reprojectCloud(int EPSG_new);   

};

#endif //POINT_CLOUD_RASTER_