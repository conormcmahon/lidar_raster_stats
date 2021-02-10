
#include <lidar_raster_stats/raster_stats_tester.h>

int main(int argc, char *argv[])
{
    int min_args = 10; 
    if(argc != min_args+1 && argc != min_args+2)
    {
        std::cout << "ERROR: Didn't receive expected number of input parameters. Received " << argc-1 << " parameters, instead of " << min_args << " or " << min_args+1 << ". Required parameters, in order:" << std::endl;
        std::cout << "  INPUT_FILENAME  - string value containing entire path to input filename, including filetype (e.g. \"/home/foo/input_cloud.pcd\")" << std::endl;
        std::cout << "  OUTPUT_FILENAME - string value containing entire path to output filename, but NOT including filetype (e.g. \"/home/foo/raster_out\")" << std::endl;
        std::cout << "  FIELD_NAME      - name of field from point cloud to use for raster outputs (e.g. \"intensity\", \"height\", \"z\"...)" << std::endl;
        std::cout << "  PIXEL_SIZE      - X/Y pixel dimensions for output raster, in output georeferenced units" << std::endl;
        std::cout << "  HISTOGRAM_MIN   - minimum bin value for histogram raster. If this and max are both set to 0, the histogram range will be separately calculated for each pixel." << std::endl;
        std::cout << "  HISTOGRAM_MAX   - maximum bin value for histogram raster. If this and min are both set to 0, the histogram range will be separately calculated for each pixel." << std::endl;
        std::cout << "  HISTOGRAM_BINS  - number of bins to be used in histogram" << std::endl;
        std::cout << "  Z_SCALE_FACTOR  - factor by which to scale Z values (e.g. to change units)" << std::endl;
        std::cout << "  SCALE_FACTOR    - factor by which to scale all values in FIELD_NAME" << std::endl;
        std::cout << "  EPSG_INPUT      - EPSG code with the CRS of the input point cloud dataset" << std::endl;
        std::cout << "  EPSG_OUTPUT     - EPSG code that point cloud data will be reprojected to prior to rasterizing. If none is specified, no reprojection occurs." << std::endl;
        return -1;
    }
    std::string pcd_filename = argv[1];
    std::string output_tif_filename = argv[2];
    std::string field_name = argv[3];
    float pixel_size = std::atof(argv[4]);
    float hist_min = std::atof(argv[5]);
    float hist_max = std::atof(argv[6]);
    int hist_bins = std::atoi(argv[7]);
    float z_scale_factor = std::atof(argv[8]);
    float scale_factor = std::atof(argv[9]);
    int EPSG = std::atoi(argv[10]);
    // Check whether a reprojection is required
    int EPSG_reproj = 0;
    if(argc == 10)
        EPSG_reproj = std::atoi(argv[11]);

    // Load input cloud
    pcl::PointCloud<pcl::PointVeg>::Ptr cloud(new pcl::PointCloud<pcl::PointVeg>);
    if (pcl::io::loadPCDFile<pcl::PointVeg> (pcd_filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << pcd_filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << std::endl;

    // Initialize Raster Generator and Build Raster Structure
    PointCloudRaster<pcl::PointVeg> rasterizer(pixel_size, pixel_size, Eigen::Vector2f::Zero()); 
    rasterizer.buildRasterStructure(cloud, EPSG, EPSG_reproj, z_scale_factor);
    // Declare Raster Objects 
    PointCloudRaster<pcl::PointVeg>::float_raster max_raster;
    PointCloudRaster<pcl::PointVeg>::float_raster min_raster;
    PointCloudRaster<pcl::PointVeg>::float_raster median_raster;
    PointCloudRaster<pcl::PointVeg>::float_raster density_raster;
    PointCloudRaster<pcl::PointVeg>::histogram_raster histogram_raster;
    // Generate Rasters
    rasterizer.generateMaxRaster     (field_name, max_raster, -9999, scale_factor);
    rasterizer.generateMinRaster     (field_name, min_raster, -9999, scale_factor);
    rasterizer.generateMedianRaster  (field_name, median_raster, -9999, scale_factor);
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
    rasterizer.generateHistogram (field_name, histogram_raster, hist_opts, -9999, scale_factor);

    // Save Rasters to Disk
    //   Set up GDAL
    GDALAllRegister();
    //   Set up GDAL Driver
    GDALDriver *driver;
    const char *pszFormat = "GTiff";
    driver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    //   Output TIFF Files
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_median.tif"), driver);
    rasterizer.outputTIF(density_raster, output_tif_filename + std::string("_density.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_histogram.tif"), driver); 

    return 1;
}

