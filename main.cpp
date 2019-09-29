#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <sys/stat.h>
#include "pcd_utils/pcd_utils.h"
#include "tsdf/tsdf.h"

bool is_path_exist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

// Loads a binary file with depth data and generates a TSDF voxel volume (5m x 5m x 5m at 1cm resolution)
// Volume is aligned with respect to the camera coordinates of the first frame (a.k.a. base frame)
int main(int argc, char * argv[]) {

  // Location of camera intrinsic file
  std::string cam_K_file = "../data/camera-intrinsics.txt";

  // Location of folder containing RGB-D frames and camera pose files
  std::string data_path = "../data/rgbd-frames";
  int base_frame_idx = 150;
  int first_frame_idx = 150;
  float num_frames = 50;

  // Image width and depth
  int im_width = 640;
  int im_height = 480;

  // Voxel grid parameters (change these to change voxel grid resolution, etc.)
  float voxel_grid_origin_x = -1.5f; // Location of voxel grid origin in base frame camera coordinates
  float voxel_grid_origin_y = -1.5f;
  float voxel_grid_origin_z = 0.5f;
  float voxel_size = 0.006f;
  float trunc_margin = voxel_size * 5;
  int voxel_grid_dim_x = 500;
  int voxel_grid_dim_y = 500;
  int voxel_grid_dim_z = 500;

  // Manual parameters
  if (argc == 3) {
    cam_K_file = argv[1];
    data_path = argv[2];
  }
  else
  {
    std::cout<<"Both cam_K_file and data_path not defined. Using default values."<<std::endl;
  }

  if(!is_path_exist(cam_K_file))
  {
    std::cerr<<cam_K_file<<" does not exist!!!"<<std::endl;
    return -1;
  }

  if(!is_path_exist(data_path))
  {
    std::cerr<<data_path<<" does not exist!!!"<<std::endl;
    return -1;
  }

  // Initialize voxel grid
  float * voxel_grid_TSDF = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  float * voxel_grid_weight = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
    voxel_grid_TSDF[i] = 1.0f;
  memset(voxel_grid_weight, 0, sizeof(float) * voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z);

  // Perform the tsdf fusion
  tsdf_fusion(cam_K_file, data_path, first_frame_idx, base_frame_idx,
    num_frames, im_height, im_width, voxel_grid_dim_x, voxel_grid_dim_y,
    voxel_grid_dim_z, voxel_grid_origin_x, voxel_grid_origin_y,
    voxel_grid_origin_z, voxel_size, trunc_margin, voxel_grid_TSDF,
    voxel_grid_weight);

  // Compute surface points from TSDF voxel grid and save to point cloud .ply file
  std::cout << "Saving surface point cloud (tsdf.pcd)..." << std::endl;
  SaveVoxelGrid2SurfacePointCloud("tsdf.pcd", voxel_grid_dim_x, voxel_grid_dim_y,
    voxel_grid_dim_z, voxel_size, voxel_grid_origin_x, voxel_grid_origin_y,
    voxel_grid_origin_z, voxel_grid_TSDF, voxel_grid_weight, 0.2f, 0.0f);

  return 0;
}
