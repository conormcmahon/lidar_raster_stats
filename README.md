# LiDAR Raster Stats

Library to convert PCL point clouds (e.g. as loaded from .PCD files) of land scenes to raster format. Calculates a set of input statistics over all points within each output raster pixel. 

# Usage

From within cloned git repository: 

```
./build/raster_stats_tester /path/to/input.pcd /path/to/output_filename intensity 8
```

- Input - Provided as full filepath to an input .pcd point cloud. The demo file used above expects 'PointVeg' type defined in [Dirt Or Leaf Library](https://github.com/conormcmahon/dirt_or_leaf). However, the library is templated so that arbitrary point types can be used in a user-written .cpp demo file. 
- Output - Provide a filepath and name prefix for the output file, but do NOT include file type. All images will be output in .tif format. Multiple different images are generated for the various statistics produced.
- Field - an argument containing the name of the field to be targetted. If a custom .cpp demo file is written, this can be an arbitrary value supported by the input cloud and point format. For the demo here, this must be one of the fields defined [here](https://github.com/conormcmahon/dirt_or_leaf/blob/master/include/dirt_or_leaf/point_veg.h). That means the following are supported: intensity, classification, height, roughness, x, y, or z. 
- Pixel Size - the width and height of each input pixel in the output image, using the units of the input point cloud. 

# Example Images

## Input Cloud

<img src="images/62072057_3d.png" width="55%">
Example image showing the input 3D point cloud for which raster are generated below. The cloud is colored by return intensity.

Currently, only four stats are included, illustrated here using the intensity information above. Histograms can also be extracted which show the distribution of point values within each pixel. 

## Maximum Image

<img src="images/raster_stats_max.png" width="40%">
Maximum within each raster pixel.

## Minimum Image

<img src="images/raster_stats_min.png" width="40%">
Minimum within each raster pixel.

## Median Image

<img src="images/raster_stats_median.png" width="40%">
Median within each raster pixel.

## Point Density Image

<img src="images/raster_stats_density.png" width="40%">
Total number of points (cloud density) within each raster pixel.

## Histogram Image

<img src="images/height_histogram.png" width="40%">
Histogram of point heights within image, aggregated at 5 ft pixel scale with 10 height bins. Red band is high vegetation, green is medium, and blue is near-ground returns. 

At another area along a major river, the following height histograms were extracted for comparison between vegetation types. All of the following histograms are with pixels aggregated at 10 m resolution with 20 bins, ranging from 0 to 20 m height. 

<img src="images/histogram_gallery_riparian.png" width="70%">
Gallery riparian trees, probably sycamore or cottonwood. 

<img src="images/histogram_willows.png" width="70%">
Lower riparian trees, probably willow.

<img src="images/histogram_chaparral.png" width="70%">
Chaparral vegetation. 
