
#ifndef POINT_CLOUD_RASTER_HPP_
#define POINT_CLOUD_RASTER_HPP_

#include <lidar_raster_stats/point_cloud_raster.h>

// Takes an input point cloud, generates mappings to a raster space
//   RASTER_INDICES contains a 2D raster where each element is a vector of all cloud points inside that pixel
//   MAP contains a list of generated i,j raster coordinates for each point in the cloud 
template <typename PointType>
PointCloudRaster<PointType>::PointCloudRaster(float pixel_width, float pixel_height, Eigen::Vector2f origin_offset)
{
    cloud_.reset(new PC);
    pixel_width_ = pixel_width;
    pixel_height_ = pixel_height;
    origin_offset_ = origin_offset;
}

template <typename PointType>
void PointCloudRaster<PointType>::buildRasterStructure(std::string filename, int EPSG, int EPSG_reproj, float scale_factor)
{
    EPSG_ = EPSG;
    // Load input cloud
    if (pcl::io::loadPCDFile<PointType> (filename, *cloud_) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return;
    }
    else 
        std::cout << "Read an input cloud with size " << cloud_->points.size() << std::endl;

    // Optionally rescale cloud in Z dimension
    if(scale_factor != 1)
        for(int i=0; i<cloud_->points.size(); i++)
            cloud_->points[i].z *= scale_factor;

    // Reproject cloud if necessary
    if(EPSG_reproj != 0)
        reprojectCloud(EPSG_reproj);

    buildRaster();
}

template <typename PointType>
void PointCloudRaster<PointType>::buildRasterStructure(PCP cloud, int EPSG, int EPSG_reproj, float scale_factor)
{
    EPSG_ = EPSG;   
    *cloud_ = *cloud;

    // Optionally rescale cloud in Z dimension
    if(scale_factor != 1)
        for(int i=0; i<cloud_->points.size(); i++)
            cloud_->points[i].z *= scale_factor;


    // Reproject cloud if necessary
    if(EPSG_reproj != 0)
        reprojectCloud(EPSG_reproj);

    buildRaster();
}


template <typename PointType>
void PointCloudRaster<PointType>::buildRaster()
{
    // Reset data, in case rasters have been built with this object before
    index_raster_.clear();
    index_map_.clear();
    // Figure out minimum and maximum coordinates within cloud
    Eigen::Vector2f min_point, max_point;  // 2D float pairs in (X,Y)
    min_point << 1e20, 1e20;
    max_point << -1e20, -1e20;
    for(std::size_t i=0; i<cloud_->points.size(); i++)
    {
        if(cloud_->points[i].x < min_point[0]) // min X
            min_point[0] = cloud_->points[i].x;
        if(cloud_->points[i].y < min_point[1]) // min Y
            min_point[1] = cloud_->points[i].y;
        if(cloud_->points[i].x > max_point[0]) // max X
            max_point[0] = cloud_->points[i].x;
        if(cloud_->points[i].y > max_point[1]) // max Y
            max_point[1] = cloud_->points[i].y;
    }
    std::cout << "Minimum point was " << min_point[0] << " (x), " << min_point[1] << " (y)" << std::setprecision(8) << std::endl;
    std::cout << "Maximum point was " << max_point[0] << " (x), " << max_point[1] << " (y)" << std::setprecision(8) << std::endl;

    // Using minimum and maximum coordinates and pixel size, figure out raster origin and size
    // *** To meet with image processing conventions, we'll keep the the image Y axis increasing DOWNWARDS, and the X axis increasing RIGHT ***
    origin_[0] = std::floor((min_point[0] - origin_offset_[0])/pixel_width_) * pixel_width_ + origin_offset_[0];
    origin_[1] = std::ceil((max_point[1] - origin_offset_[1])/pixel_height_) * pixel_height_ + origin_offset_[1];
    width_ = int(std::ceil((max_point[0] - origin_[0])/pixel_width_))+1;
    height_ = int(std::ceil((origin_[1] - min_point[1])/pixel_height_))+1;
    std::cout << "Raster size was " << width_ << " x " << height_ << ", with pixel size " << pixel_width_ << " x " << pixel_height_ << std::setprecision(8) << std::endl;
    std::cout << "Raster origin is at " << origin_[0] << " (x), " << origin_[1] << " (y)" << std::setprecision(8) << std::endl;
    // Initialize output raster of indices using the width and height found above
    index_raster_.resize(width_, std::vector<std::vector<int> >(height_));

    // Fill the raster of indices 
    for(std::size_t i=0; i<cloud_->points.size(); i++)
    {
        int x_index, y_index;
        // *** Again, X increases RIGHT (easting) and Y increases DOWN (-northing) ***
        x_index = int(std::floor((cloud_->points[i].x - origin_[0])/pixel_width_));
        y_index = int(std::floor((origin_[1] - cloud_->points[i].y)/pixel_height_));
        if(x_index > width_-1 || y_index > height_-1 || x_index < 0 || y_index < 0)
        {
            std::cout << "WARNING - index out of bounds for " << x_index << " " << y_index << " " << cloud_->points[i].x << " " << cloud_->points[i].y << " " << i << std::setprecision(8) << std::endl;
            continue;
        }
        index_raster_[x_index][y_index].push_back(i);
    }

    index_map_.resize(cloud_->points.size(), std::make_pair(-1,-1));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
            for(std::size_t k=0; k<index_raster_[i][j].size(); k++)
            {
                if(index_raster_[i][j][k] >= 0)
                    index_map_[index_raster_[i][j][k]] = std::make_pair(i,j);
            }
}

template <typename PointType>
PointCloudRaster<PointType>::~PointCloudRaster()
{
    GDALDestroy();
}

template <typename PointType>
bool PointCloudRaster<PointType>::checkRasterInitialization()
{
    if(index_raster_.size() < 1)
    {
        std::cout << "WARNING - input raster has width < 1!";
        return false;
    }
    if(index_raster_[0].size() < 1)
    {
        std::cout << "WARNING - input raster has height < 1!";
        return false;
    }
    for(std::size_t i; i<index_raster_.size(); i++)
        if(index_raster_[i].size() != index_raster_[0].size())
        {
            std::cout << "WARNING: input raster has height varying by column. Height for col=0: " << index_raster_[0].size() << "; Height for col=" << i << ": " << index_raster_[i].size();
            return false;
        }
    return true;
}

template <typename PointType>
template <typename DataType>
bool PointCloudRaster<PointType>::checkRasterInitialization(std::vector<std::vector<DataType> > const &raster)
{
    if(raster.size() < 1)
    {
        std::cout << "WARNING: input raster has width < 1!";
        return false;
    }
    if(raster[0].size() < 1)
    {
        std::cout << "WARNING: input raster has height < 1!";
        return false;
    }
    for(std::size_t i; i<raster.size(); i++)
        if(raster[i].size() != raster[0].size())
        {
            std::cout << "WARNING: input raster has height varying by column. Height for col=0: " << raster[0].size() << "; Height for col=" << i << ": " << raster[i].size();
            return false;
        }
    return true;
}


// Simple functions to return structural information on image layout
template <typename PointType>
Eigen::Vector2f PointCloudRaster<PointType>::getOrigin()
{
    return origin_;
}
template <typename PointType>
Eigen::Vector2i PointCloudRaster<PointType>::getImageSize()
{
    Eigen::Vector2i size;
    size << width_, height_;
    return size;
}
template <typename PointType>
Eigen::Vector2f PointCloudRaster<PointType>::getPixelSize()
{
    Eigen::Vector2f pixel_size;
    pixel_size << pixel_width_, pixel_height_;
    return pixel_size;
}


template <typename PointType>
void PointCloudRaster<PointType>::reprojectCloud(int EPSG_new)
{  
    std::cout << "Reprojecting input cloud from EPSG:" << EPSG_ << " to EPSG:" << EPSG_new << std::endl;

    // Set up reprojection from initial to new CRS
    OGRSpatialReference in_SRS;
    in_SRS.importFromEPSG(EPSG_);
    OGRSpatialReference out_SRS;
    out_SRS.importFromEPSG(EPSG_new);
    OGRCoordinateTransformation *transformer = OGRCreateCoordinateTransformation(&in_SRS, &out_SRS);

    for(int i=0; i<cloud_->points.size(); i++)
    {
        // Get reprojected XYZ coordinates
        double x = cloud_->points[i].x;
        double y = cloud_->points[i].y;
        double z = cloud_->points[i].z;
        transformer->Transform(1, &x, &y, &z);
        
        // Assign new coordinates to PCL point 
        cloud_->points[i].x = x;
        cloud_->points[i].y = y;
        cloud_->points[i].z = z; 
    }
    EPSG_ = EPSG_new;  
}


template <typename PointType>
void PointCloudRaster<PointType>::generateMinRaster(std::string field_name, float_raster &raster_out, float default_value, float scale_factor)
{
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<float>(height_, 0));
    std::cout << "Generating minimum value raster for input cloud with size " << cloud_->points.size() << " using field " << field_name << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "WARNING: Requested to generate minimum from empty raster; exiting.";
        return;
    }
    // Build a 2D matrix of bools the same size as image, with TRUE for filled pixels and FALSE for empty pixels
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
        {
            // Skip empty raster pixels
            if(index_raster_[i][j].size() < 1)
                continue;
            float min = pcl::getFieldValue<PointType,float>( cloud_->points[index_raster_[i][j][0]], field_name ) * scale_factor;                // initialize to first point value in pixel
            for(std::size_t k=1; k<index_raster_[i][j].size(); k++)
            {
                float new_value = pcl::getFieldValue<PointType,float>( cloud_->points[index_raster_[i][j][k]], field_name ) * scale_factor;
                if(new_value < min)
                    min = new_value;
            }
            raster_out[i][j] = min;
            empty_cells[i][j] = false;
        }
    // To DO - hole filling, interpolation, etc. 
} 
template <typename PointType>
void PointCloudRaster<PointType>::generateMaxRaster(std::string field_name, float_raster &raster_out, float default_value, float scale_factor)
{
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<float>(height_, 0));
    std::cout << "Generating maximum value raster for input cloud with size " << cloud_->points.size() << " using field " << field_name << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "WARNING: Requested to generate maximum from empty raster; exiting.";
        return;
    }
    // Build a 2D matrix of bools the same size as image, with TRUE for filled pixels and FALSE for empty pixels
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
        {
            // Skip empty raster pixels
            if(index_raster_[i][j].size() < 1)
                continue;
            float max = pcl::getFieldValue<PointType,float>( cloud_->points[index_raster_[i][j][0]], field_name ) * scale_factor;                // initialize to first point value in pixel
            for(std::size_t k=1; k<index_raster_[i][j].size(); k++)
            {
                float new_value = pcl::getFieldValue<PointType,float>( cloud_->points[index_raster_[i][j][k]], field_name ) * scale_factor;
                if(new_value > max)
                    max = new_value;
            }
            raster_out[i][j] = max;
            empty_cells[i][j] = false;
        }
    // To DO - hole filling, interpolation, etc. 
} 
template <typename PointType>
void PointCloudRaster<PointType>::generateMedianRaster(std::string field_name, float_raster &raster_out, float default_value, float scale_factor)
{ 
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<float>(height_, 0));
    std::cout << "Generating median value raster for input cloud with size " << cloud_->points.size() << " using field " << field_name << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "WARNING: Requested to generate median from empty raster; exiting.";
        return;
    } 
    // Fill image with data
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
        {
            // Skip empty raster pixels
            if(index_raster_[i][j].size() < 1)
                continue;
            // Build and sort list of values
            std::vector<float> values(index_raster_[i][j].size(), 0);
            for(std::size_t k=0; k<index_raster_[i][j].size(); k++)
                values[k] = ( pcl::getFieldValue<PointType,float>(cloud_->points[index_raster_[i][j][k]], field_name) * scale_factor );
            std::sort(values.begin(), values.end());
            // Get median from sorted list            
            int median_index = floor(values.size()/2);
            if(values.size() % 2 == 1) // if an odd number of values in pixel, take middle value
                raster_out[i][j] = values[median_index];
            else // if an even, positive number of values in pixel, need to average two values for median
            {
                raster_out[i][j] = (values[median_index] + values[median_index-1]) / 2; 
            }
            empty_cells[i][j] = false;
        }
    // To DO - hole filling, interpolation, etc.  
} 
template <typename PointType>
void PointCloudRaster<PointType>::generatePercentileRaster(std::string field_name, float_raster &raster_out, float percentile, float default_value, float scale_factor)
{ 
    if(percentile > 1 || percentile < 0)
    {
        std::cout << "WARNING: Requested to generate percentile raster, but selected percentile of " << percentile << " is not within expected range of 0.0 to 1.0, and so no raster will be generated." << std::endl;
        return;
    }
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<float>(height_, 0));
    std::cout << "Generating " << percentile << "th percentile raster for input cloud with size " << cloud_->points.size() << " using field " << field_name << std::setprecision(8) << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "WARNING: Requested to generate percentile from empty raster; exiting.";
        return;
    } 
    // Fill image with data
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
        {
            // Skip empty raster pixels
            if(index_raster_[i][j].size() < 1)
                continue;
            // Build and sort list of values
            std::vector<float> values(index_raster_[i][j].size(), 0);
            for(std::size_t k=0; k<index_raster_[i][j].size(); k++)
                values[k] = ( pcl::getFieldValue<PointType,float>(cloud_->points[index_raster_[i][j][k]], field_name) * scale_factor );
            std::sort(values.begin(), values.end());
            // Get target percentile from sorted list:
            int index = floor(values.size()*percentile);
            if(percentile == 1)
                raster_out[i][j] = values[index-1];
            else
                raster_out[i][j] = values[index];
            empty_cells[i][j] = false;
        }
    // To DO - hole filling, interpolation, etc.  
} 

template <typename PointType>
void PointCloudRaster<PointType>::generateDensityRaster(float_raster &raster_out)
{
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<float>(height_, 0));
    std::cout << "Generating point density raster for input cloud with size " << cloud_->points.size() << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "WARNING: Requested to generate density from empty raster; exiting.";
        return;
    }
    // Fill image with data
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<index_raster_[i].size(); j++)
        {
            raster_out[i][j] = index_raster_[i][j].size();
            empty_cells[i][j] = (raster_out[i][j] == 0);
        }
    // To DO - hole filling, interpolation, etc. 
} 

template <typename PointType>
void PointCloudRaster<PointType>::generateHistogram(std::string field_name, histogram_raster &raster_out, HistogramOptions opt, float default_value, float scale_factor)
{
    std::cout << "Generating histogram raster with " << opt.num_bins << " bins for input cloud with size " << cloud_->points.size() << std::endl;
    if(!checkRasterInitialization())
    {
        std::cout << "  WARNING: Requested to generate histogram from empty raster; exiting without building raster.";
        return;
    }
    // Resize Output Image
    raster_out.clear();
    raster_out.resize(width_, std::vector<std::vector<float> >(height_, std::vector<float>(opt.num_bins+2, 0)));
    // Debugging based on options
    if(opt.scaled_by_pixel)
        std::cout << "  Histogram will be generated with different minima/maxima bounds for each pixel, depending on the local distribution of values." << std::endl;
    else if(opt.min_max_specified)
        std::cout << "  Histogram will be generated with a single fixed minimum and maximum across all pixels. Min: " << opt.min_value << ";  Max: " << opt.max_value << std::endl;
    if(opt.scaled_by_pixel && opt.min_max_specified)
    {
        std::cout << "  WARNING: The user specified a min/max value for histogram but also specified this should be scaled for each pixel. Pixelwise scaling will be used and specified global values discarded." << std::endl; 
        opt.min_max_specified = false;
    }
    if(!(opt.scaled_by_pixel) && !(opt.min_max_specified))
    {
        opt.min_value = 10e10;
        opt.max_value = -10e10;
        for(int i=0; i<cloud_->points.size(); i++)
        {
            float value = pcl::getFieldValue<PointType,float>(cloud_->points[i], field_name) * scale_factor;
            if(value < opt.min_value)
                opt.min_value = value;
            if(value > opt.max_value)
                opt.max_value = value;
        }
        std::cout << "  Histogram will be generated with a single fixed minimum and maximum across all pixels. Min: " << opt.min_value << ";  Max: " << opt.max_value << std::endl;
    }
    // Build a 2D matrix of bools the same size as image, with TRUE for filled pixels and FALSE for empty pixels
    std::vector<std::vector<bool>> empty_cells(width_, std::vector<bool>(height_, true));
    // Populate output raster with data
    for(std::size_t i=0; i<width_; i++)
        for(std::size_t j=0; j<height_; j++)
        {
            // Skip empty raster pixels
            if(index_raster_[i][j].size() < 1)
                continue;
            // Build and sort list of values
            std::vector<float> values;
            for(std::size_t k=0; k<index_raster_[i][j].size(); k++)
                values.push_back( pcl::getFieldValue<PointType,float>(cloud_->points[index_raster_[i][j][k]], field_name) * scale_factor );
            std::sort(values.begin(), values.end());
            // Assign min and max values for this histogram, if they vary by pixel
            if(opt.scaled_by_pixel)
            {
                // Cannot infer bin width if number of requested bins is greater than number of points
                if(values.size() < opt.num_bins) 
                    continue;
                opt.min_value = values[0]; // min field value
                opt.max_value = values[values.size()-1]; // max field value
            }
            raster_out[i][j][0] = opt.min_value;
            raster_out[i][j][1] = opt.max_value;
            for(std::size_t k=0; k<values.size(); k++)
            {
                int hist_pos;
                if(values[k] < opt.min_value)
                    hist_pos = 0;
                else if(values[k] > opt.max_value)
                    hist_pos = opt.num_bins - 1; 
                else 
                {
                    float pos_temp = (values[k] - opt.min_value) / (opt.max_value - opt.min_value);
                    hist_pos = int(floor(pos_temp * (opt.num_bins-1)));
                }
                hist_pos += 2; // because first two bands are the min/max values
                raster_out[i][j][hist_pos]++;
            }
            for(std::size_t bin=2; bin<opt.num_bins+2; bin++)
            {
                raster_out[i][j][bin] /= values.size();
            }
            empty_cells[i][j] = false;  
        }
    // To DO - hole filling, interpolation, etc. 
}


template <typename PointType>
void PointCloudRaster<PointType>::outputTIF(float_raster const &image, std::string filename, GDALDriver *driver)
{ 
    std::cout << "Asked to save a file to " << filename << std::endl;

    if(!checkRasterInitialization(image))
    {
        std::cout << "WARNING: asked to save a faulty image file to " << filename << ". Returning without saving.";
        return;
    }
    // the following based on https://gdal.org/tutorials/raster_api_tut.html#getting-dataset-information

    // Image dimensions
    unsigned int height = image.size();
    unsigned int width = image[0].size();

    // Create GDAL driver for file IO

    char **papszMetadata;
    if( driver == NULL )
        exit( 1 );
    papszMetadata = driver->GetMetadata();
    if( !CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
    {
        std::cout << "  Driver " << " does not support Create() method. Returning without saving output cloud.";
        return;
    }

    // Create image on disk
    GDALDataset *poDstDS;
    char **papszOptions = NULL;
    poDstDS = driver->Create( filename.c_str(), image.size(), image[0].size(), 1, GDT_Float32,
                                papszOptions );
                                
    // Set up transform

    //   Geographic transform (x_min, x_pixel_width, 0, y_min, 0, -y_pixel_width)
    double adfGeoTransform[6] = { origin_[0], pixel_width_, 0, origin_[1], 0, -pixel_height_}; //
    poDstDS->SetGeoTransform( adfGeoTransform );

    //   Well-known Text for base coordinate system
//    std::string fish = "PROJCS[\"NAD_1983_CORS96_StatePlane_California_VI_FIPS_0406_Ft_US\",GEOGCS[\"NAD83(CORS96)\",DATUM[\"NAD83_Continuously_Operating_Reference_Station_1996\",SPHEROID[\"GRS 1980\",6378137,298.257222101004,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"1133\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"6783\"]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"latitude_of_origin\",32.1666666666667],PARAMETER[\"central_meridian\",-116.25],PARAMETER[\"standard_parallel_1\",32.7833333333333],PARAMETER[\"standard_parallel_2\",33.8833333333333],PARAMETER[\"false_easting\",6561666.66666667],PARAMETER[\"false_northing\",1640416.66666667],UNIT[\"US survey foot\",0.304800609601219,AUTHORITY[\"EPSG\",\"9003\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH]]";
    OGRSpatialReference oSRS;
    char *pszSRS_WKT = NULL;
//    oSRS.SetUTM( 11, TRUE );
//    oSRS.SetWellKnownGeogCS( "NAD83" );
    oSRS.importFromEPSG(EPSG_);
    oSRS.exportToWkt( &pszSRS_WKT );
    poDstDS->SetProjection( pszSRS_WKT );
    CPLFree( pszSRS_WKT );
    // Fill image data structure in memory
    float abyRaster[height*width]; 
    unsigned int ind = 0;
    for(unsigned int i=0; i<width; i++)
        for(unsigned int j=0; j<height; j++)
        {
            abyRaster[ind] = image[j][i];
            ind += 1;
        }
    // Populate image with data on disk
    GDALRasterBand *poBand;
    poBand = poDstDS->GetRasterBand(1);
    CPLErr err = poBand->RasterIO( GF_Write, 0, 0, height, width,
                                   abyRaster, height, width, GDT_Float32, 0, 0 ); 
    // Close and save file 
    GDALClose( (GDALDatasetH) poDstDS ); 
}


template <typename PointType>
void PointCloudRaster<PointType>::outputTIFMultiband(histogram_raster const &image, std::string filename, GDALDriver *driver)
{ 
    std::cout << "Asked to save a file to " << filename << std::endl;

    if(!checkRasterInitialization(image))
    {
        std::cout << "WARNING: asked to save a faulty image file to " << filename << ". Returning without saving.";
        return;
    }
    // the following based on https://gdal.org/tutorials/raster_api_tut.html#getting-dataset-information

    // Image dimensions
    unsigned int height = image.size();
    unsigned int width = image[0].size();
    unsigned int num_bands = image[0][0].size();

    if( driver == NULL )
        exit( 1 );
    char **papszMetadata;
    papszMetadata = driver->GetMetadata();
    if( !CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
    {
        std::cout << "  Driver " << " does not support Create() method. Returning without saving output cloud.";
        return;
    }

    // Create image on disk
    GDALDataset *poDstDS;
    char **papszOptions = NULL;
    poDstDS = driver->Create( filename.c_str(), image.size(), image[0].size(), num_bands, GDT_Float32,
                                papszOptions );
    
    // Set up transform
    //   Geographic transform (x_min, x_pixel_width, 0, y_min, 0, -y_pixel_width)
    double adfGeoTransform[6] = { origin_[0], pixel_width_, 0, origin_[1], 0, -pixel_height_}; //
    poDstDS->SetGeoTransform( adfGeoTransform );
    //   Well-known Text for base coordinate system
//    std::string fish = "PROJCS[\"NAD_1983_CORS96_StatePlane_California_VI_FIPS_0406_Ft_US\",GEOGCS[\"NAD83(CORS96)\",DATUM[\"NAD83_Continuously_Operating_Reference_Station_1996\",SPHEROID[\"GRS 1980\",6378137,298.257222101004,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"1133\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"6783\"]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"latitude_of_origin\",32.1666666666667],PARAMETER[\"central_meridian\",-116.25],PARAMETER[\"standard_parallel_1\",32.7833333333333],PARAMETER[\"standard_parallel_2\",33.8833333333333],PARAMETER[\"false_easting\",6561666.66666667],PARAMETER[\"false_northing\",1640416.66666667],UNIT[\"US survey foot\",0.304800609601219,AUTHORITY[\"EPSG\",\"9003\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH]]";
    OGRSpatialReference oSRS;
    char *pszSRS_WKT = NULL;
//    oSRS.SetUTM( 11, TRUE );
//    oSRS.SetWellKnownGeogCS( "NAD83" );
    oSRS.importFromEPSG(EPSG_);
    oSRS.exportToWkt( &pszSRS_WKT );
    poDstDS->SetProjection( pszSRS_WKT );
    CPLFree( pszSRS_WKT );
    // Output Raster Bands to Disk
    for(int band=0; band<num_bands; band++)
    {
        // Fill image data structure in memory
        float abyRaster[height*width]; 
        unsigned int ind = 0;
        for(unsigned int i=0; i<width; i++)
            for(unsigned int j=0; j<height; j++)
            {
                abyRaster[ind] = image[j][i][band];
                ind += 1;
            }
        // Populate image with data on disk
        GDALRasterBand *poBand;
        poBand = poDstDS->GetRasterBand(band+1);
        CPLErr err = poBand->RasterIO( GF_Write, 0, 0, height, width,
                                       abyRaster, height, width, GDT_Float32, 0, 0 );
    }
    // Close and save file  
    GDALClose( (GDALDatasetH) poDstDS ); 
}

#endif //POINT_CLOUD_RASTER_HPP_