#include "pcd_utils.h"
#include <cmath>

// Compute surface points from TSDF voxel grid and save points to point cloud file
void SaveVoxelGrid2SurfacePointCloud(const std::string &file_name, int voxel_grid_dim_x,
  int voxel_grid_dim_y, int voxel_grid_dim_z, float voxel_size, float voxel_grid_origin_x,
  float voxel_grid_origin_y, float voxel_grid_origin_z, float * voxel_grid_TSDF,
  float * voxel_grid_weight, float tsdf_thresh, float weight_thresh) {

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Create point cloud content for ply file
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; i++) {

    // If TSDF value of voxel is less than some threshold, add voxel coordinates to point cloud
    if (std::fabs(voxel_grid_TSDF[i]) < tsdf_thresh && voxel_grid_weight[i] > weight_thresh) {

      // Compute voxel indices in int for higher positive number range
      int z = floor(i / (voxel_grid_dim_x * voxel_grid_dim_y));
      int y = floor((i - (z * voxel_grid_dim_x * voxel_grid_dim_y)) / voxel_grid_dim_x);
      int x = i - (z * voxel_grid_dim_x * voxel_grid_dim_y) - (y * voxel_grid_dim_x);

      // Convert voxel indices to float and store in cloud
      pcl::PointXYZ point;
      point.x = voxel_grid_origin_x + (float) x * voxel_size;
      point.y = voxel_grid_origin_y + (float) y * voxel_size;
      point.z = voxel_grid_origin_z + (float) z * voxel_size;

      cloud.points.push_back(point);
    }
  }

  cloud.width  = 1;
  cloud.height = cloud.points.size();
  pcl::io::savePCDFileASCII (file_name, cloud);
}
