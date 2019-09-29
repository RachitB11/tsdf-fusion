#ifndef PCD_UTILS_
#define PCD_UTILS_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>

// Compute surface points from TSDF voxel grid and save points to point cloud file
void SaveVoxelGrid2SurfacePointCloud(const std::string &file_name, int voxel_grid_dim_x,
  int voxel_grid_dim_y, int voxel_grid_dim_z, float voxel_size, float voxel_grid_origin_x,
  float voxel_grid_origin_y, float voxel_grid_origin_z, float * voxel_grid_TSDF,
  float * voxel_grid_weight, float tsdf_thresh, float weight_thresh);



#endif
