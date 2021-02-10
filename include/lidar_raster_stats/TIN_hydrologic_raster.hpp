
#ifndef TIN_HYRDOLOGIC_RASTER_HPP_
#define TIN_HYRDOLOGIC_RASTER_HPP_

#include <lidar_raster_stats/TIN_hydrologic_raster.h>

template <typename PointType, typename ChannelTypeFlat>
TINHydrologicRaster<PointType, ChannelTypeFlat>::TINHydrologicRaster(float pixel_width, float pixel_height, int horz_sample_density, int vert_sample_density, Eigen::Vector2f origin_offset) : 
    TINRaster<PointType>(pixel_width, pixel_height, horz_sample_density, vert_sample_density, origin_offset)
{
    data_search_tree_.reset(new PTree);
}


template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateTerrainInfo(std::string slope_field, std::string aspect_field)
{
    this->TINRaster<PointType>::generateTerrainInfo(slope_field, aspect_field);
    data_search_tree_->setInputCloud(this->PointCloudRaster<PointType>::cloud_);
}


template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateStreamDistances(const OGRFlowlinesSettings &settings, std::string horz_dist_field, std::string vert_dist_field)
{
    checkSettingsValidity(horz_dist_field, vert_dist_field);

    flowlines_.loadFromSHP(settings);
    flowlines_.getSearchTree(flowlines_search_tree_);

    generateStreamDistancesInternal(horz_dist_field, vert_dist_field);
}


template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateStreamDistances(std::string flowlines_pcd_filename, std::string horz_dist_field, std::string vert_dist_field)
{
    checkSettingsValidity(horz_dist_field, vert_dist_field);

    PTreeP target_tree(new PTree); 
    target_tree->setInputCloud(this->PointCloudRaster<PointType>::cloud_);
    PointType point;
    flowlines_.loadFromPCD(flowlines_pcd_filename, point, this->PointCloudRaster<PointType>::cloud_, target_tree, 2000);
    flowlines_.getSearchTree(flowlines_search_tree_);

    generateStreamDistancesInternal(horz_dist_field, vert_dist_field);
}


template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateStreamDistances(CCP flowlines_cloud, std::string horz_dist_field, std::string vert_dist_field)
{    
    checkSettingsValidity(horz_dist_field, vert_dist_field);

    // Verify that flowlines cloud is non-empty
    if(flowlines_cloud->points.size() < 1)
    {
        std::cout << "WARNING: asked to generate stream distances but there are no stream points provided." << std::endl;
        return;
    }

    flowlines_.loadFromCloud(flowlines_cloud);
    flowlines_.getSearchTree(flowlines_search_tree_);
   
    generateStreamDistancesInternal(horz_dist_field, vert_dist_field);
}

template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateStreamDistancesInternal(std::string horz_dist_field, std::string vert_dist_field)
{
    flowlines_.populateHeightFromTIN(this->TINRaster<PointType>::TIN_);
    for(int i=0; i<this->PointCloudRaster<PointType>::cloud_->points.size(); i++)
    {
        Eigen::Vector2f stream_distances = flowlines_.getPointDistance(this->PointCloudRaster<PointType>::cloud_->points[i]);
        pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], 
                                    horz_dist_field, 
                                    float(stream_distances[0]), true );
        pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], 
                                    vert_dist_field, 
                                    float(stream_distances[1]), true );  
    }
}

template <typename PointType, typename ChannelTypeFlat>
bool TINHydrologicRaster<PointType, ChannelTypeFlat>::checkSettingsValidity(std::string vert_dist_field, std::string horz_dist_field)
{  
    // Verify that target data cloud is non-empty
    if(this->PointCloudRaster<PointType>::cloud_->points.size() < 1)
    {
        std::cout << "WARNING: asked to generate stream distances but the target data cloud is empty." << std::endl;
        return false;
    }
    // Check that requested slope field exists and is of the correct size (Float) or the assignment will fail:
    if(!pcl::checkFieldType<PointType, float>(vert_dist_field))
    {
        std::cout << "Field " << vert_dist_field << " is not a valid field for vertical stream distance data. Field names are case sensitive, and the field size must be of std::float." << std::endl;
        return false;
    }
    // Check that requested aspect field exists and is of the correct size (Float) or the assignment will fail:
    if(!pcl::checkFieldType<PointType, float>(horz_dist_field))
    {
        std::cout << "Field " << horz_dist_field << " is not a valid field for horizontal stream distance data. Field names are case sensitive, and the field size must be of std::float." << std::endl;
        return false;
    }
    return true;
}

template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::writeFlowlinesCloud(std::string filename, bool binary)
{
    flowlines_.writeFlowlinesCloud(filename, binary);
}



#endif //TIN_HYRDOLOGIC_RASTER_HPP_