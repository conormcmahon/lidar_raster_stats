
#include <lidar_raster_stats/raster_stats_tester.h>

int main(int argc, char *argv[])
{
    std::string pcd_filename = argv[1];
    std::string output_tif_filename = argv[2];
    std::string field_name = argv[3];
    float pixel_size = std::atof(argv[4]);
    float hist_min = std::atof(argv[5]);
    float hist_max = std::atof(argv[6]);
    int hist_bins = std::atoi(argv[7]);

    // Load input cloud
    pcl::PointCloud<pcl::PointVeg>::Ptr cloud(new pcl::PointCloud<pcl::PointVeg>);
    if (pcl::io::loadPCDFile<pcl::PointVeg> (pcd_filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << pcd_filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << std::endl;

    // Declare Raster Objects
    PointCloudRaster<pcl::PointVeg> rasterizer(cloud, pixel_size, pixel_size, Eigen::Vector2f::Zero(), true);  
    std::vector<std::vector<float> > max_raster;
    std::vector<std::vector<float> > min_raster;
    std::vector<std::vector<float> > median_raster;
    std::vector<std::vector<float> > density_raster;
    std::vector<std::vector<std::vector<float> > > histogram_raster;
    // Generate Rasters
    rasterizer.generateMaxRaster    (field_name, max_raster, -9999);
    rasterizer.generateMinRaster    (field_name, min_raster, -9999);
    rasterizer.generateMedianRaster (field_name, median_raster, -9999);
    rasterizer.generateDensityRaster (density_raster);
    // Populate Histogram Options and Generate Histogram Raster
    HistogramOptions hist_opts;
    hist_opts.num_bins = hist_bins;
    hist_opts.min_value = hist_min;
    hist_opts.max_value = hist_max;
    if(hist_opts.min_value != hist_opts.max_value)
    {   
        hist_opts.min_max_specified = true;
        hist_opts.scaled_by_pixel = false;
    }
    else
    {
        hist_opts.min_max_specified = false;
        hist_opts.scaled_by_pixel = true;
    }
    rasterizer.generateVegHeightHistogram (field_name, histogram_raster, hist_opts, -9999);
    // Save Rasters to Disk
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_max.tif"));
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_min.tif"));
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_median.tif"));
    rasterizer.outputTIF(density_raster, output_tif_filename + std::string("_density.tif"));
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_histogram.tif"));

}

