
#ifndef TIN_RASTER_
#define TIN_RASTER_

#include <dirt_or_leaf/las_tin.hpp>

template <typename PointType>
class TINRaster : public PointCloudRaster<PointType>
{
public:
    typedef typename pcl::PointCloud<PointType> PC;
    typedef typename pcl::PointCloud<PointType>::Ptr PCP;

    TINRaster(float pixel_width, float pixel_height, int horz_sample_density, int vert_sample_density, Eigen::Vector2f origin_offset=Eigen::Vector2f::Zero());

    // Load Data, optionally reproject, and build raster
    //    Load cloud from existing PCL object in memory
    void buildRasterStructure(PCP cloud, int EPSG, int EPSG_reproj=0);
    //    Load cloud from .PCD file 
    void buildRasterStructure(std::string filename, int EPSG, int EPSG_reproj=0);

protected:
    // Data Objects
    LAS_TIN<PointType> TIN_;
    PCP source_cloud_;

    int horz_sample_density_;
    int vert_sample_density_;

    void sampleTIN();

};

#endif //TIN_RASTER_