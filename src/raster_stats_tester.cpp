
#include <lidar_raster_stats/raster_stats_tester.h>

int main(int argc, char *argv[])
{
    std::string pcd_filename = argv[1];
    std::string output_tif_filename = argv[2];
    std::string field_name = argv[3];
    float pixel_size = std::atof(argv[4]);

    // Load input cloud
    pcl::PointCloud<pcl::PointVeg>::Ptr cloud(new pcl::PointCloud<pcl::PointVeg>);
    if (pcl::io::loadPCDFile<pcl::PointVeg> (pcd_filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << pcd_filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << std::endl;

    PointCloudRaster<pcl::PointVeg> rasterizer(cloud, pixel_size, pixel_size, Eigen::Vector2f::Zero(), true);  
    std::vector<std::vector<float> > max_raster;
    std::vector<std::vector<float> > min_raster;
    std::vector<std::vector<float> > median_raster;
    std::vector<std::vector<float> > density_raster;
    rasterizer.generateMaxRaster    (field_name, max_raster, -9999);
    rasterizer.generateMinRaster    (field_name, min_raster, -9999);
    rasterizer.generateMedianRaster (field_name, median_raster, -9999);
    rasterizer.generateDensityRaster (density_raster);
    rasterizer.outputTIF(max_raster, output_tif_filename + std::string("_max.tif"));
    rasterizer.outputTIF(min_raster, output_tif_filename + std::string("_min.tif"));
    rasterizer.outputTIF(median_raster, output_tif_filename + std::string("_median.tif"));
    rasterizer.outputTIF(density_raster, output_tif_filename + std::string("_density.tif"));


/*
    std::cout << " -------------------------------------------- " << std::endl;

    std::string tif_filename = "/mnt/c/Users/conor/Documents/test.tif";
    GDALDataset *poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen( tif_filename.c_str(), GA_ReadOnly );

    double        adfGeoTransform[6];
    printf( "Driver: %s/%s\n",
            poDataset->GetDriver()->GetDescription(),
            poDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );
    printf( "Size is %dx%dx%d\n",
            poDataset->GetRasterXSize(), poDataset->GetRasterYSize(),
            poDataset->GetRasterCount() );
    if( poDataset->GetProjectionRef()  != NULL )
        printf( "Projection is `%s'\n", poDataset->GetProjectionRef() );
    if( poDataset->GetGeoTransform( adfGeoTransform ) == CE_None )
    {
        printf( "Origin = (%.6f,%.6f)\n",
                adfGeoTransform[0], adfGeoTransform[3] );
        printf( "Pixel Size = (%.6f,%.6f)\n",
                adfGeoTransform[1], adfGeoTransform[5] );
        printf( "Last values = (%.6f,%.6f)\n", adfGeoTransform[2], adfGeoTransform[4]);
    }

    std::cout << " -------------------------------------------- " << std::endl;

    GDALRasterBand  *poBand;
    int             nBlockXSize, nBlockYSize;
    int             bGotMin, bGotMax;
    double          adfMinMax[2];
    poBand = poDataset->GetRasterBand( 1 );
    poBand->GetBlockSize( &nBlockXSize, &nBlockYSize );
    printf( "Block=%dx%d Type=%s, ColorInterp=%s\n",
            nBlockXSize, nBlockYSize,
            GDALGetDataTypeName(poBand->GetRasterDataType()),
            GDALGetColorInterpretationName(
                poBand->GetColorInterpretation()) );
    adfMinMax[0] = poBand->GetMinimum( &bGotMin );
    adfMinMax[1] = poBand->GetMaximum( &bGotMax );
    if( ! (bGotMin && bGotMax) )
        GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
    printf( "Min=%.3f, Max=%.3f\n", adfMinMax[0], adfMinMax[1] );
    if( poBand->GetOverviewCount() > 0 )
        printf( "Band has %d overviews.\n", poBand->GetOverviewCount() );
    if( poBand->GetColorTable() != NULL )
        printf( "Band has a color table with %d entries.\n",
                poBand->GetColorTable()->GetColorEntryCount() );

*/
}

