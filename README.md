Point Cloud Compression

Using the ROS recording function we are able to seperate the Point Cloud data into a
depth map and color map. Then the depth map is compressed frame by frame using lz4 compression.
Then the color map is losslessly encoding through opencv that provides the user the ability to 
change encoding algorithims. Then the decompress node will broadcast an uncompressed depth map
and color map. This is then merged to recreate the point cloud in real time.
