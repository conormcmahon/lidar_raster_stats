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

<img src="images/62072057_3d.png" width="55%">
Example image showing the input 3D point cloud for which raster are generated below.

Currently, only four stats are included:

<img src="images/raster_stats_max.png" width="40%">
Maximum within each raster pixel.

<img src="images/raster_stats_min.png" width="40%">
Minimum within each raster pixel.

<img src="images/raster_stats_median.png" width="40%">
Median within each raster pixel.

<img src="images/raster_stats_density.png" width="40%">
Total number of points (cloud density) within each raster pixel.

