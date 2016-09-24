# PCL-RealSense
RealSense - Point Cloud Library Bridge (VS2015)

The code is grabbing depth and RGB image from a RealSense camera, maps them to fix the misalignment between depth and RGB image. 
Then, transfers it to PCL for visualization (and possible processing) purpose. 
Due to intensive point copy operations, it is limited to 1~2 fps 

# Usage 
- Install PCL and RealSense SDK
- Set following parameters as PATH

RSSDK_PATH (i.e. c:\Program Files (x86)\Intel\RSSDK)

PCL_PATH (i.e. C:\Program Files\PCL 1.7.2)

- If you install 3rd party libraries in a different location than PCL/3rdParty, modify directories

# Hints 

- The code has been tested with SR300 and R200 under Windows 10.
- In point cloud, real world coordinates have been used. If you need, you can scale it.

# Screenshots 

## RGB  |  Depth
![alt tag](https://github.com/dBeker/PCL-RealSense/blob/master/images/rgb.png) | ![alt tag](https://github.com/dBeker/PCL-RealSense/blob/master/images/depth.png)

## Point Cloud
![alt tag](https://github.com/dBeker/PCL-RealSense/blob/master/images/pcl.png)
