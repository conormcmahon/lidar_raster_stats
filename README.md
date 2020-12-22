# LiDAR Raster Stats

Library to convert PCL point clouds (e.g. as loaded from .PCD files) of land scenes to raster format. Calculates a set of input statistics over all points within each output raster pixel. 

# Usage

From within cloned git repository: 

```
./build/raster_stats_tester /path/to/input.pcd /path/to/output_filename intensity 8
```

- Input - Provided as full filepath to an input .pcd point cloud. The demo file used above expects 'VegPoint' type defined in [Dirt Or Leaf Library](https://github.com/conormcmahon/dirt_or_leaf). However, the library is templated so that arbitrary point types can be used in a user-written .cpp demo file. 
- Output - Provide a filepath and name prefix for the output file, but do NOT include file type. All images will be output in .tif format. Multiple different images are generated for the various statistics produced.

# Example Images

Currently, only four stats are included:

[Max Image](images/raster_stats_max.png){:height="50%" width="50%"}
Maximum within each raster pixel.

[Min Image](images/raster_stats_min.png){:height="50%" width="50%"}
Minimum within each raster pixel.

[Median Image](images/raster_stats_median.png){:height="50%" width="50%"}
Median within each raster pixel.

[Density Image](images/raster_stats_density.png){:height="50%" width="50%"}
Total number of points (cloud density) within each raster pixel.
