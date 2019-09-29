# Volumetric TSDF Fusion of Multiple Depth Maps

Refactored the code in [Andy Zeng's Project](https://github.com/andyzeng/tsdf-fusion) to build with CMake on Ubuntu 16.04 with PCL 1.7
and CUDA 9.

![Teaser](tsdf.png?raw=true)

CUDA/C++ code to fuse multiple registered depth maps into a projective truncated signed distance function (TSDF) voxel volume, which can then be used to create high quality 3D surface meshes and point clouds.

## Demo
This demo fuses 50 registered depth maps from directory `data/rgbd-frames` into a projective TSDF voxel volume, and creates a 3D surface point cloud `tsdf.pcd`
**Note**: Input depth maps should be saved in format: 16-bit PNG, depth in millimeters.

## Build and run demo
Build the demo
```
mkdir build
cd build/
cmake ..
make
```

Run the demo
```
cd bin/
./main
```
