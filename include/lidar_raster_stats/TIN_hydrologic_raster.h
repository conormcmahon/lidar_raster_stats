
#ifndef TIN_HYDROLOGIC_RASTER_
#define TIN_HYDROLOGIC_RASTER_

#include <dirt_or_leaf/las_tin.hpp>
#include <dirt_or_leaf/las_conversions.hpp>
#include <riparian_hydrology_pcl/flowlines_pcl.hpp>
#include <gdal/ogrsf_frmts.h>

#include "lidar_raster_stats/TIN_raster.hpp"

template <typename PointType, typename ChannelTypeFlat>
class TINHydrologicRaster : public TINRaster<PointType>
{
public:
    typedef typename pcl::PointCloud<ChannelTypeFlat> CC;               // Point Cloud for Channel Point Type
    typedef typename pcl::PointCloud<ChannelTypeFlat>::Ptr CCP;         // Pointer to Point Cloud for Channel Point Type
    typedef typename pcl::KdTreeFLANN<ChannelTypeFlat> CTree;           // KDTree for Channel Point Type
    typedef typename pcl::KdTreeFLANN<ChannelTypeFlat>::Ptr CTreeP;     // Pointer to KDTree for Channel Point Type
    typedef typename pcl::KdTreeFLANN<PointType> PTree;           // KDTree for Input Point Type
    typedef typename pcl::KdTreeFLANN<PointType>::Ptr PTreeP;     // Pointer to KDTree for Input Point Type

    TINHydrologicRaster(float pixel_width, float pixel_height, int horz_sample_density, int vert_sample_density, Eigen::Vector2f origin_offset=Eigen::Vector2f::Zero());

    void generateTerrainInfo(std::string slope_field="slope", std::string aspect_field="aspect");

    // Generate Distances to Stream Flowlines from Each Point in Cloud
    void generateStreamDistances(const OGRFlowlinesSettings &settings, std::string horz_dist_field="dist_from_stream", std::string vert_dist_field="height_over_stream");
    // Output Flowlines cloud to disk
    void writeFlowlinesCloud(std::string filename, bool binary=true);

protected:
    // Data Objects
    FlowlinesPCL<ChannelTypeFlat> flowlines_;
    CTreeP flowlines_search_tree_;
    PTreeP data_search_tree_;

    void sampleOGRSegmentToCloud(OGRFeature &segment, float linear_density, std::string channel_order_field, std::string channel_name_field);
};

#endif //TIN_HYDROLOGIC_RASTER_