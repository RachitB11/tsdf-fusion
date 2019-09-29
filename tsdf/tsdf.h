#ifndef TSDF_
#define TSDF_

#include <string>


void tsdf_fusion(std::string cam_K_file, std::string data_path, int first_frame_idx,
  int base_frame_idx, float num_frames, int im_height, int im_width,
  int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
  float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z,
  float voxel_size, float trunc_margin, float * voxel_grid_TSDF,
  float * voxel_grid_weight);

#endif
