#include "tsdf.h"
#include "../utils/cuda_utils.h"
#include "../utils/utils.h"
#include <iomanip>
#include <vector>
#include <iostream>
#include <sstream>

// CUDA kernel function to integrate a TSDF voxel volume given depth images
__global__
void Integrate(float * cam_K, float * cam2base, float * depth_im,
               int im_height, int im_width, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
               float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z, float voxel_size, float trunc_margin,
               float * voxel_grid_TSDF, float * voxel_grid_weight) {

  int pt_grid_z = blockIdx.x;
  int pt_grid_y = threadIdx.x;

  for (int pt_grid_x = 0; pt_grid_x < voxel_grid_dim_x; ++pt_grid_x) {

    // Convert voxel center from grid coordinates to base frame camera coordinates
    float pt_base_x = voxel_grid_origin_x + pt_grid_x * voxel_size;
    float pt_base_y = voxel_grid_origin_y + pt_grid_y * voxel_size;
    float pt_base_z = voxel_grid_origin_z + pt_grid_z * voxel_size;

    // Convert from base frame camera coordinates to current frame camera coordinates
    float tmp_pt[3] = {0};
    tmp_pt[0] = pt_base_x - cam2base[0 * 4 + 3];
    tmp_pt[1] = pt_base_y - cam2base[1 * 4 + 3];
    tmp_pt[2] = pt_base_z - cam2base[2 * 4 + 3];
    float pt_cam_x = cam2base[0 * 4 + 0] * tmp_pt[0] + cam2base[1 * 4 + 0] * tmp_pt[1] + cam2base[2 * 4 + 0] * tmp_pt[2];
    float pt_cam_y = cam2base[0 * 4 + 1] * tmp_pt[0] + cam2base[1 * 4 + 1] * tmp_pt[1] + cam2base[2 * 4 + 1] * tmp_pt[2];
    float pt_cam_z = cam2base[0 * 4 + 2] * tmp_pt[0] + cam2base[1 * 4 + 2] * tmp_pt[1] + cam2base[2 * 4 + 2] * tmp_pt[2];

    if (pt_cam_z <= 0)
      continue;

    int pt_pix_x = roundf(cam_K[0 * 3 + 0] * (pt_cam_x / pt_cam_z) + cam_K[0 * 3 + 2]);
    int pt_pix_y = roundf(cam_K[1 * 3 + 1] * (pt_cam_y / pt_cam_z) + cam_K[1 * 3 + 2]);
    if (pt_pix_x < 0 || pt_pix_x >= im_width || pt_pix_y < 0 || pt_pix_y >= im_height)
      continue;

    float depth_val = depth_im[pt_pix_y * im_width + pt_pix_x];

    if (depth_val <= 0 || depth_val > 6)
      continue;

    float diff = depth_val - pt_cam_z;

    if (diff <= -trunc_margin)
      continue;

    // Integrate
    int volume_idx = pt_grid_z * voxel_grid_dim_y * voxel_grid_dim_x + pt_grid_y * voxel_grid_dim_x + pt_grid_x;
    float dist = fmin(1.0f, diff / trunc_margin);
    float weight_old = voxel_grid_weight[volume_idx];
    float weight_new = weight_old + 1.0f;
    voxel_grid_weight[volume_idx] = weight_new;
    voxel_grid_TSDF[volume_idx] = (voxel_grid_TSDF[volume_idx] * weight_old + dist) / weight_new;
  }
}


void tsdf_fusion(std::string cam_K_file, std::string data_path, int first_frame_idx,
  int base_frame_idx, float num_frames, int im_height, int im_width,
  int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
  float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z,
  float voxel_size, float trunc_margin, float * voxel_grid_TSDF,
  float * voxel_grid_weight)
{
  // Setup the different arrays
  float cam_K[3 * 3];
  float base2world[4 * 4];
  float cam2base[4 * 4];
  float cam2world[4 * 4];
  float depth_im[im_height * im_width];

  // Read camera intrinsics
  std::vector<float> cam_K_vec = LoadMatrixFromFile(cam_K_file, 3, 3);
  std::copy(cam_K_vec.begin(), cam_K_vec.end(), cam_K);

  // Read base frame camera pose
  std::ostringstream base_frame_prefix;
  base_frame_prefix << std::setw(6) << std::setfill('0') << base_frame_idx;
  std::string base2world_file = data_path + "/frame-" + base_frame_prefix.str() + ".pose.txt";
  std::vector<float> base2world_vec = LoadMatrixFromFile(base2world_file, 4, 4);
  std::copy(base2world_vec.begin(), base2world_vec.end(), base2world);

  // Invert base frame camera pose to get world-to-base frame transform
  float base2world_inv[16] = {0};
  invert_matrix(base2world, base2world_inv);

  // Load variables to GPU memory
  float * gpu_voxel_grid_TSDF;
  float * gpu_voxel_grid_weight;
  cudaMalloc(&gpu_voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float));
  cudaMalloc(&gpu_voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());
  cudaMemcpy(gpu_voxel_grid_TSDF, voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_voxel_grid_weight, voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyHostToDevice);
  checkCUDA(__LINE__, cudaGetLastError());
  float * gpu_cam_K;
  float * gpu_cam2base;
  float * gpu_depth_im;
  cudaMalloc(&gpu_cam_K, 3 * 3 * sizeof(float));
  cudaMemcpy(gpu_cam_K, cam_K, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMalloc(&gpu_cam2base, 4 * 4 * sizeof(float));
  cudaMalloc(&gpu_depth_im, im_height * im_width * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());

  // Loop through each depth frame and integrate TSDF voxel grid
  for (int frame_idx = first_frame_idx; frame_idx < first_frame_idx + (int)num_frames; ++frame_idx) {

    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;

    // // Read current frame depth
    std::string depth_im_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
    ReadDepth(depth_im_file, im_height, im_width, depth_im);

    // Read base frame camera pose
    std::string cam2world_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    std::vector<float> cam2world_vec = LoadMatrixFromFile(cam2world_file, 4, 4);
    std::copy(cam2world_vec.begin(), cam2world_vec.end(), cam2world);

    // Compute relative camera pose (camera-to-base frame)
    multiply_matrix(base2world_inv, cam2world, cam2base);

    cudaMemcpy(gpu_cam2base, cam2base, 4 * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_depth_im, depth_im, im_height * im_width * sizeof(float), cudaMemcpyHostToDevice);
    checkCUDA(__LINE__, cudaGetLastError());

    std::cout << "Fusing: " << depth_im_file << std::endl;

    Integrate <<< voxel_grid_dim_z, voxel_grid_dim_y >>>(gpu_cam_K, gpu_cam2base, gpu_depth_im,
                                                         im_height, im_width, voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z,
                                                         voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z, voxel_size, trunc_margin,
                                                         gpu_voxel_grid_TSDF, gpu_voxel_grid_weight);
  }

  // Load TSDF voxel grid from GPU to CPU memory
  cudaMemcpy(voxel_grid_TSDF, gpu_voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(voxel_grid_weight, gpu_voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyDeviceToHost);
  checkCUDA(__LINE__, cudaGetLastError());
}
