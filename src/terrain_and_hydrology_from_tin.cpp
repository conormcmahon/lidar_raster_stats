
#include <lidar_raster_stats/terrain_and_hydrology_from_tin.h>

int main(int argc, char *argv[])
{
    int min_args = 12;
    if(argc != min_args+1 && argc != min_args+2)
    {
        std::cout << "ERROR: Didn't receive expected number of input parameters. Received " << argc-1 << " parameters, instead of " << min_args << " or " << min_args+1 << ". Required parameters, in order:" << std::endl;
        std::cout << "  INPUT_FILENAME     - string value containing entire path to input filename, including filetype (e.g. \"/home/foo/input_cloud.pcd\")" << std::endl;
        std::cout << "  FLOWLINES_FILENAME - string value containing entire path to filename for stream flowlines, in .SHP format" << std::endl;
        std::cout << "  FLOWLINES_LAYER    - string value containing layer name within Shapefile loaded for flowlines" << std::endl;
        std::cout << "  OUTPUT_FILENAME    - string value containing entire path to output filename, but NOT including filetype (e.g. \"/home/foo/raster_out\")" << std::endl;
        std::cout << "  FIELD_NAME         - name of field from point cloud to use for raster outputs (e.g. \"intensity\", \"height\", \"z\"...)" << std::endl;
        std::cout << "  PIXEL_SIZE         - X/Y pixel dimensions for output raster, in output georeferenced units" << std::endl;
        std::cout << "  SAMPLE_DENSITY     - density at which to sample TIN within each raster pixel - e.g. a value of 10 will result in 10x10=100 points being equally spaced vertically in each pixel" << std::endl;
        std::cout << "  HISTOGRAM_MIN      - minimum bin value for histogram raster. If this and max are both set to 0, the histogram range will be separately calculated for each pixel." << std::endl;
        std::cout << "  HISTOGRAM_MAX      - maximum bin value for histogram raster. If this and min are both set to 0, the histogram range will be separately calculated for each pixel." << std::endl;
        std::cout << "  HISTOGRAM_BINS     - number of bins to be used in histogram" << std::endl;
        std::cout << "  SCALE_FACTOR       - factor by which to scale all values in FIELD_NAME " << std::endl;
        std::cout << "  EPSG_INPUT         - EPSG code with the CRS of the input point cloud dataset" << std::endl;
        std::cout << "  EPSG_OUTPUT        - EPSG code that point cloud data will be reprojected to prior to rasterizing. If none is specified, no reprojection occurs." << std::endl;
        return -1;
    }
    std::string pcd_filename = argv[1];
    std::string flowlines_filename = argv[2];
    std::string flowlines_layer = argv[3];
    std::string output_tif_filename = argv[4];
    std::string field_name = argv[5];
    float pixel_size = std::atof(argv[6]);
    int sample_density = std::atoi(argv[7]);
    float hist_min = std::atof(argv[8]);
    float hist_max = std::atof(argv[9]);
    int hist_bins = std::atoi(argv[10]);
    float scale_factor = std::atof(argv[11]);
    int EPSG = std::atoi(argv[12]);
    // Check whether a reprojection is required
    int EPSG_reproj = 0;
    if(argc == min_args+2)
        EPSG_reproj = std::atoi(argv[13]);

    // Load input cloud
    pcl::PointCloud<pcl::Point2DGround>::Ptr cloud(new pcl::PointCloud<pcl::Point2DGround>);
    if (pcl::io::loadPCDFile<pcl::Point2DGround> (pcd_filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << pcd_filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << std::endl;

    // Initialize Raster Generator and Build Raster Structure
    TINHydrologicRaster<pcl::Point2DGround, pcl::PointChannel> rasterizer(pixel_size, pixel_size, sample_density, sample_density, Eigen::Vector2f::Zero());  
    rasterizer.buildRasterStructure(cloud, EPSG, EPSG_reproj);
    rasterizer.generateTerrainInfo();
    if(field_name.compare("z")==0 || field_name.compare("height")==0)
        rasterizer.saveResampledCloud(output_tif_filename + ".pcd", scale_factor, true);
    else
        rasterizer.saveResampledCloud(output_tif_filename + ".pcd", 1, true);
    // Flowlines Information
    OGRFlowlinesSettings settings;
    settings.flowlines_filename_ = flowlines_filename;
    settings.layer_name_ = flowlines_layer;
    settings.shp_name_field_ = "GNIS_Nm";
    settings.shp_order_field_ = "StrmOrd";
    settings.filter_by_order_ = true;
    settings.filter_by_name_ = false;
    settings.channel_order_threshold_ = 3;
    settings.pcl_order_field_ = "stream_order";
    settings.linear_point_density_ = 1;
    settings.EPSG_in_ = EPSG;
    settings.EPSG_reproj_ = EPSG_reproj;
    rasterizer.generateStreamDistances(settings);
    rasterizer.writeFlowlinesCloud(output_tif_filename+ + "_flowlines.pcd", true);

    // Declare Raster Objects 
    PointCloudRaster<pcl::Point2DGround>::float_raster max_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster min_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster median_raster;
    PointCloudRaster<pcl::Point2DGround>::float_raster density_raster;
    PointCloudRaster<pcl::Point2DGround>::histogram_raster histogram_raster;
    // Generate Rasters
    rasterizer.generateMaxRaster     (field_name, max_raster, scale_factor, -9999);
    rasterizer.generateMinRaster     (field_name, min_raster, scale_factor, -9999);
    rasterizer.generateMedianRaster  (field_name, median_raster, scale_factor, -9999);
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
    rasterizer.generateHeightHistogram (field_name, histogram_raster, hist_opts, scale_factor, -9999);

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

