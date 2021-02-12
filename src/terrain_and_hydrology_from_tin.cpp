
#include <lidar_raster_stats/terrain_and_hydrology_from_tin.h>

int main(int argc, char *argv[])
{
    int min_args = 8;
    if(argc != min_args+1 && argc != min_args+2)
    {
        std::cout << "ERROR: Didn't receive expected number of input parameters. Received " << argc-1 << " parameters, instead of " << min_args << " or " << min_args+1 << ". Required parameters, in order:" << std::endl;
        std::cout << "  INPUT_FILENAME     - string value containing entire path to input filename, including filetype (e.g. \"/home/foo/input_cloud.pcd\")" << std::endl;
        std::cout << "  FLOWLINES_FILENAME - string value containing entire path to filename for stream flowlines, in .PCD format" << std::endl;
        std::cout << "  OUTPUT_FILENAME    - string value containing entire path to output filename, but NOT including filetype (e.g. \"/home/foo/raster_out\")" << std::endl;
        std::cout << "  PIXEL_SIZE         - X/Y pixel dimensions for output raster, in output georeferenced units" << std::endl;
        std::cout << "  SAMPLE_DENSITY     - density at which to sample TIN within each raster pixel - e.g. a value of 10 will result in 10x10=100 points being equally spaced vertically in each pixel" << std::endl;
        std::cout << "  HISTOGRAM_BINS     - number of bins to be used in histogram" << std::endl;
        std::cout << "  Z_SCALE_FACTOR     - factor by which to scale all values in the Z dimension (for when a reprojection doesn't include Z unit changes)" << std::endl;
        std::cout << "  EPSG_INPUT         - EPSG code with the CRS of the input point cloud dataset" << std::endl;
        std::cout << "  EPSG_OUTPUT        - EPSG code that point cloud data will be reprojected to prior to rasterizing. If none is specified, no reprojection occurs." << std::endl;
        return -1;
    }
    std::string pcd_filename = argv[1];
    std::string flowlines_filename = argv[2];
    std::string output_tif_filename = argv[3];
    float pixel_size = std::atof(argv[4]);
    int sample_density = std::atoi(argv[5]);
    int hist_bins = std::atoi(argv[6]);
    float scale_factor = std::atof(argv[7]);
    int EPSG = std::atoi(argv[8]);
    // Check whether a reprojection is required
    int EPSG_reproj = 0;
    if(argc == min_args+2)
        EPSG_reproj = std::atoi(argv[9]);

    // Load input cloud
    pcl::PointCloud<pcl::Point2DGround>::Ptr cloud(new pcl::PointCloud<pcl::Point2DGround>);
    if (pcl::io::loadPCDFile<pcl::Point2DGround> (pcd_filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << pcd_filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << " from file " << pcd_filename << std::endl;

    // Initialize Raster Generator and Build Raster Structure
    TINHydrologicRaster<pcl::Point2DGround, pcl::PointChannel> rasterizer(pixel_size, pixel_size, sample_density, sample_density, Eigen::Vector2f::Zero());  
    rasterizer.buildRasterStructure(cloud, EPSG, EPSG_reproj, scale_factor);
    rasterizer.generateTerrainInfo();
    rasterizer.generateStreamDistances(flowlines_filename);
    rasterizer.writeFlowlinesCloud(output_tif_filename+ + "_flowlines.pcd", true);
    rasterizer.saveResampledCloud(output_tif_filename + ".pcd", true);

    // Histogram Options
    HistogramOptions hist_opts;
    hist_opts.num_bins = hist_bins;
    hist_opts.min_max_specified = true;
    hist_opts.scaled_by_pixel = false;

    // Set up GDAL
    GDALAllRegister();
    //   Set up GDAL Driver
    GDALDriver *driver;
    const char *pszFormat = "GTiff";
    driver = GetGDALDriverManager()->GetDriverByName(pszFormat);

    // Declare Raster Objects 
    PointCloudRaster<pcl::Point2DGround>::float_raster max_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster min_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster median_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster density_raster;
    PointCloudRaster<pcl::Point2DGround>::histogram_raster histogram_raster;

    // Generate and Output Rasters

    // ---- Slope ---- 
    //    Generate Rasters
    rasterizer.generateMaxRaster     ("slope", max_raster, -9999);
    rasterizer.generateMinRaster     ("slope", min_raster, -9999);
    rasterizer.generateMedianRaster  ("slope", median_raster, -9999);
    rasterizer.generateDensityRaster (density_raster);
    hist_opts.min_value = 0;
    hist_opts.max_value = M_PI/2;
    rasterizer.generateHistogram ("slope", histogram_raster, hist_opts, -9999);
    //    Output TIFF Files 
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_slope_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_slope_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_slope_median.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_slope_histogram.tif"), driver); 
    rasterizer.outputTIF(density_raster, output_tif_filename + std::string("_density.tif"), driver);

    // ---- Aspect ----
    rasterizer.generateMaxRaster     ("aspect", max_raster, -9999);
    rasterizer.generateMinRaster     ("aspect", min_raster, -9999);
    rasterizer.generateMedianRaster  ("aspect", median_raster, -9999);
    hist_opts.min_value = 0;
    hist_opts.max_value = M_PI;
    rasterizer.generateHistogram ("aspect", histogram_raster, hist_opts, -9999);
    //    Output TIFF Files 
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_aspect_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_aspect_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_aspect_median.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_aspect_histogram.tif"), driver); 

    // ---- Height Over Stream ----
    rasterizer.generateMaxRaster     ("height_over_stream", max_raster, -9999);
    rasterizer.generateMinRaster     ("height_over_stream", min_raster, -9999);
    rasterizer.generateMedianRaster  ("height_over_stream", median_raster, -9999);
    hist_opts.min_value = 0;
    hist_opts.max_value = 1000;
    rasterizer.generateHistogram ("height_over_stream", histogram_raster, hist_opts, -9999);
    //    Output TIFF Files 
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_streamheight_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_streamheight_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_streamheight_median.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_streamheight_histogram.tif"), driver); 

    // ---- Distance From Stream ----
    rasterizer.generateMaxRaster     ("dist_from_stream", max_raster, -9999);
    rasterizer.generateMinRaster     ("dist_from_stream", min_raster, -9999);
    rasterizer.generateMedianRaster  ("dist_from_stream", median_raster, -9999);
    hist_opts.min_value = 0;
    hist_opts.max_value = 1000;
    rasterizer.generateHistogram ("dist_from_stream", histogram_raster, hist_opts, -9999);
    //    Output TIFF Files 
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_streamdist_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_streamdist_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_streamdist_median.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_streamdist_histogram.tif"), driver); 

    // ---- Raw Z ----
    rasterizer.generateMaxRaster     ("z", max_raster, -9999);
    rasterizer.generateMinRaster     ("z", min_raster, -9999);
    rasterizer.generateMedianRaster  ("z", median_raster, -9999);
    hist_opts.min_value = 0;
    hist_opts.max_value = 0;
    hist_opts.min_max_specified = false;
    hist_opts.scaled_by_pixel = false;
    rasterizer.generateHistogram ("z", histogram_raster, hist_opts, -9999);
    //    Output TIFF Files 
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_z_max.tif"), driver);
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_z_min.tif"), driver);
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_z_median.tif"), driver);
    rasterizer.outputTIFMultiband(histogram_raster, output_tif_filename + std::string("_z_histogram.tif"), driver); 
    
    return 1;
}

