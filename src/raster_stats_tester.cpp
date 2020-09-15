
#include <lidar_raster_stats/raster_stats_tester.h>

int main()
{
    // Load input cloud
    std::string filename = "/home/conor/lidar_data/62072057.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return (-1);
    }
    else 
    std::cout << "Read an input cloud with size " << cloud->points.size() << std::endl;

    point_cloud_rasterizer::index_raster raster;
    point_cloud_rasterizer::index_map map;
    point_cloud_rasterizer::rasterize(cloud, raster, map, 3.28084, 3.28084, Eigen::Vector2f::Zero(), true);
    point_cloud_rasterizer::printDensityCloud(raster, "/home/conor/lidar_data/output/density.pcd");
}

