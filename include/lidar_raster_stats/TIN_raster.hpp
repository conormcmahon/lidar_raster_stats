
#ifndef TIN_RASTER_HPP_
#define TIN_RASTER_HPP_

#include <lidar_raster_stats/TIN_raster.h>

template <typename PointType>
TINRaster<PointType>::TINRaster(float pixel_width, float pixel_height, int horz_sample_density, int vert_sample_density, Eigen::Vector2f origin_offset) : 
    PointCloudRaster<PointType>(pixel_width, pixel_height, origin_offset)
{
    source_cloud_.reset(new PC);
    TIN_ = LAS_TIN<PointType>();
    horz_sample_density_ = horz_sample_density;
    vert_sample_density_ = vert_sample_density;
}

// Sample 
//   Load cloud from filename
template <typename PointType>
void TINRaster<PointType>::buildRasterStructure(PCP cloud, int EPSG, int EPSG_reproj)
{
    this->PointCloudRaster<PointType>::buildRasterStructure(cloud, EPSG, EPSG_reproj);
    sampleTIN();
}
//    Load cloud from .PCD file 
template <typename PointType>
void TINRaster<PointType>::buildRasterStructure(std::string filename, int EPSG, int EPSG_reproj)
{
    this->PointCloudRaster<PointType>::buildRasterStructure(filename, EPSG, EPSG_reproj);
    sampleTIN();
}


// Sample 
template <typename PointType>
void TINRaster<PointType>::sampleTIN()
{
    // Save source cloud before creating new resampled cloud from TIN
    *source_cloud_ = *(this->PointCloudRaster<PointType>::cloud_);
    // Build Triangulated Irregular Network surface
    TIN_.setInputCloud(this->PointCloudRaster<PointType>::cloud_);
    TIN_.generateTIN();
    TIN_.saveTIN("/mnt/d/serdp/test.ply", false);

    PCP resampled_cloud(new PC());
    // iterate over pixels in raster
    for(int i=0; i<this->PointCloudRaster<PointType>::width_; i++)
        for(int j=0; j<this->PointCloudRaster<PointType>::height_; j++)
            // iterate over sub-pixel samples
            for(int i_sub=0; i_sub<horz_sample_density_; i_sub++)
                for(int j_sub=0; j_sub<vert_sample_density_; j_sub++)
                {
                    // Sample a point
                    PointType point;
                    point.x = this->PointCloudRaster<PointType>::origin_[0] + (i + float(i_sub)/horz_sample_density_) * this->PointCloudRaster<PointType>::pixel_width_;
                    point.y = this->PointCloudRaster<PointType>::origin_[1] - (j + float(j_sub)/vert_sample_density_) * this->PointCloudRaster<PointType>::pixel_height_; // NOTE vertical dimension in image increases downwards
                    point.z = 0;

                    point.z = TIN_.interpolateTIN(point);
                    resampled_cloud->points.push_back(point);
                }
    resampled_cloud->height = 1;
    resampled_cloud->width = resampled_cloud->points.size();

    pcl::PCDWriter writer;
    writer.write<PointType>("/mnt/d/serdp/test.pcd", *resampled_cloud, true);

    // Generate raster data structure on new cloud
    this->PointCloudRaster<PointType>::buildRasterStructure(resampled_cloud, this->PointCloudRaster<PointType>::EPSG_);
}


#endif //TIN_RASTER_HPP_