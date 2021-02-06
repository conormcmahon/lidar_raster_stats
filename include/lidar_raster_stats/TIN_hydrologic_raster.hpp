
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
    flowlines_.loadFromSHP(settings);
    flowlines_.populateHeightFromTIN(this->TINRaster<PointType>::TIN_);
    for(int i=0; i<this->PointCloudRaster<PointType>::cloud_->points.size(); i++)
    {
        Eigen::Vector2f stream_distances = flowlines_.getPointDistance(this->PointCloudRaster<PointType>::cloud_->points[i]);
        if(pcl::checkFieldType<PointType, float>(horz_dist_field))
            pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], 
                                     horz_dist_field, 
                                     stream_distances[0] );
        if(pcl::checkFieldType<PointType, float>(vert_dist_field))
            pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], 
                                     vert_dist_field, 
                                     stream_distances[1] );
    }
}

template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::writeFlowlinesCloud(std::string filename, bool binary)
{
    flowlines_.writeFlowlinesCloud(filename, binary);
}


/*
template <typename PointType, typename ChannelTypeFlat>
void TINHydrologicRaster<PointType, ChannelTypeFlat>::generateStreamDistances(CCP flowlines_cloud, std::string vert_dist_field, std::string horz_dist_field)
{    
    *flowlines_ = *flowlines_cloud;
    flowlines_search_tree_->setInputCloud(flowlines_);

    std::cout << "Populating input cloud with slope and aspect information." << std::endl;
    // Verify that target data cloud is non-empty
    if(this->PointCloudRaster<PointType>::cloud_->points.size() < 1)
    {
        std::cout << "WARNING: asked to generate stream distances but the target data cloud is empty." << std::endl;
        return;
    }
    // Verify that flowlinse cloud is non-empty
    if(flowlines_->points.size() < 1)
    {
        std::cout << "WARNING: asked to generate stream distances but there are no stream points provided." << std::endl;
        return;
    }
    // Check that requested slope field exists and is of the correct size (Float) or the assignment will fail:
    if(!this->PointCloudRaster<PointType>::template pcl::isFieldFloat<PointType, float>(vert_dist_field))
    {
        std::cout << "Field " << vert_dist_field << " is not a valid field for vertical stream distance data. Field names are case sensitive, and the field size must be of std::float." << std::endl;
        return;
    }
    // Check that requested aspect field exists and is of the correct size (Float) or the assignment will fail:
    if(!this->PointCloudRaster<PointType>::template pcl::isFieldFloat<PointType, float>(horz_dist_field))
    {
        std::cout << "Field " << horz_dist_field << " is not a valid field for horizontal stream distance data. Field names are case sensitive, and the field size must be of std::float." << std::endl;
        return;
    }

    for(int i=0; i<this->PointCloudRaster<PointType>::cloud_->points.size(); i++)
    {
        // Get nearest-neighbor flowlines point
        std::vector<int> nearest_indices;
        std::vector<float> dists_squared;
        ChannelTypeFlat point_flat;
        point_flat = pcl::copyPoint3D<PointType, ChannelTypeFlat>(this->PointCloudRaster<PointType>::cloud_->points[i]);
        flowlines_search_tree_->nearestKSearch(point_flat, 1, nearest_indices, dists_squared);
        float vertical_distance = point_flat.z - flowlines_->points[nearest_indices[0]].z;
        float horizontal_distance = sqrt(dists_squared[0]);

        // Populate cloud with this data
        pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], vert_dist_field, vertical_distance );
        pcl::assignValueToField( this->PointCloudRaster<PointType>::cloud_->points[i], horz_dist_field, horizontal_distance );
    }  
}
*/

#endif //TIN_HYRDOLOGIC_RASTER_HPP_