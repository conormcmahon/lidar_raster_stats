
#include <lidar_raster_stats/point_cloud_rasterizer.h>

template <typename CloudType>
void point_cloud_rasterizer::rasterize(CloudType cloud, index_raster& raster_indices, index_map& map, float pixel_width, float pixel_height, Eigen::Vector2f origin, bool debugging)
{
    // Figure out minimum and maximum coordinates within cloud
    Eigen::Vector2f min_point, max_point;  // 2D float pairs in (X,Y)
    min_point << 1e20, 1e20;
    max_point << -1e20, -1e20;
    for(std::size_t i=0; i<cloud->points.size(); i++)
    {
        if(cloud->points[i].x < min_point[0]) // min X
            min_point[0] = cloud->points[i].x;
        if(cloud->points[i].y < min_point[1]) // min Y
            min_point[1] = cloud->points[i].y;
        if(cloud->points[i].x > max_point[0]) // max X
            max_point[0] = cloud->points[i].x;
        if(cloud->points[i].y > max_point[1]) // max Y
            max_point[1] = cloud->points[i].y;
    }
    if(debugging)
    {
        std::cout << "Minimum point was " << min_point[0] << " (x), " << min_point[1] << " (y)" << std::endl;
        std::cout << "Maximum point was " << max_point[0] << " (x), " << max_point[1] << " (y)" << std::endl;
    }

    // Using minimum and maximum coordinates and pixel size, figure out raster origin and size
    Eigen::Vector2f raster_origin;
    raster_origin[0] = std::floor((min_point[0] - origin[0])/pixel_width) * pixel_width + origin[0];
    raster_origin[1] = std::floor((min_point[1] - origin[1])/pixel_height) * pixel_height + origin[1];
    int raster_width = int(std::ceil((max_point[0] - raster_origin[0])/pixel_width))+1;
    int raster_height = int(std::ceil((max_point[1] - raster_origin[1])/pixel_height))+1;
    if(debugging)
    {
        std::cout << "Raster size was " << raster_width << " x " << raster_height << std::endl;
        std::cout << "Raster origin is at " << raster_origin[0] << " (x), " << raster_origin[1] << " (y)" << std::endl;
    }
    // Initialize output raster of indices using the width and height found above
    raster_indices.resize(raster_width, std::vector<std::vector<int> >(raster_height));

    // Fill the raster of indices 
    for(std::size_t i=0; i<cloud->points.size(); i++)
    {
        int x_index, y_index;
        x_index = int(std::floor((cloud->points[i].x - raster_origin[0])/pixel_width));
        y_index = int(std::floor((cloud->points[i].y - raster_origin[1])/pixel_height));
        if(x_index > raster_width-1 || y_index > raster_height-1 || x_index < 0 || y_index < 0)
        {
            std::cout << "WARNING - index out of bounds for " << x_index << " " << y_index << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << i << std::endl;
            continue;
        }
        raster_indices[x_index][y_index].push_back(i);
    }

    map.resize(cloud->points.size(), std::make_pair(-1,-1));
    for(std::size_t i=0; i<raster_width; i++)
        for(std::size_t j=0; j<raster_height; j++)
            for(std::size_t k=0; k<raster_indices[i][j].size(); k++)
            {
                if(raster_indices[i][j][k] >= 0)
                    map[raster_indices[i][j][k]] = std::make_pair(i,j);
            }
}

int point_cloud_rasterizer::printDensityCloud(index_raster& raster, std::string filename)
{
    if(raster.size() < 1)
    {
        std::cout << "[printDensityCloud] ERROR - input raster is empty." << std::endl;
        return -1;
    }
    if(raster[0].size() < 1)
    {
        std::cout << "[printDensityCloud] ERROR - input raster has width, but has no height." << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    int width = raster.size();
    int height = raster[0].size();
    float avg_pt_density = 0;
    for(std::size_t i=0; i<width; i++)
        for(std::size_t j=0; j<height; j++)
        {
            pcl::PointXYZI point; 
            point.x = i;
            point.y = j;
            point.z = 0;
            point.intensity = raster[i][j].size();
            cloud.points.push_back(point);
            avg_pt_density += ((raster[i])[j]).size();
        }
    avg_pt_density /= (height*width);
    std::cout << "Average raster density: " << avg_pt_density << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>(filename, cloud, true);
    return 1;
}