# lidar_raster_stats

Library to convert PCL point clouds (e.g. as loaded from .PCD files) of land scenes to raster format.

Inputs:
- Cloud file name to be loaded and processed
- Output file name to be saved
- Field to use for processing (e.g. height, Z, intensity...)
- Types of rasters to produce (currently: min, max, mean, median, point density)
- Raster pixel size (height/width) in output units

Outputs files as GTif using GDAL library. 
