//
// Created by ian on 26.11.2021.
//


#include "../include/pclhelper.h"
#include <math.h>

void __global__
toXYZKernel(float fx,
            float fy,
            float cx,
            float cy,
            int w,
            int h,
            float* Z,
            float* z,
            float* x,
            float* y)
{
    int xId = blockIdx.x * blockDim.x + threadIdx.x;
    int yId = blockIdx.y * blockDim.y + threadIdx.y;

    int index = yId * w + xId;
    if (index < (w * h) && !isnan(Z[index]) && !isinf(Z[index]) && Z[index] > 0) {
        x[index] = (float(xId - cx) * Z[index] / fx) / 1000.0;
        y[index] = (float(yId - cy) * Z[index] / fy) / 1000.0;
        z[index] = Z[index] / 1000.0;
    }
}

void __global__
toXYZRGBKernel(float fx,
               float fy,
               float cx,
               float cy,
               int w,
               int h,
               float* Z,
               unsigned char* rgb,
               pcl::PointXYZRGB* points)
{
    int xId = blockIdx.x * blockDim.x + threadIdx.x;
    int yId = blockIdx.y * blockDim.y + threadIdx.y;

    int index = yId * w + xId;
    if (index < (w * h) && !isnan(Z[index]) && !isinf(Z[index]) && Z[index] > 0) {

        points[index].z = Z[index] * 0.001;
        points[index].x = (float(xId - cx) / fx) * points[index].z;
        points[index].y = (float(yId - cy) / fy) * points[index].z;
        int rgb_index = (3840 * yId) + (3 * xId);
        points[index].r = rgb[rgb_index];
        points[index].g = rgb[rgb_index + 1];
        points[index].b = rgb[rgb_index + 2];
    }
}

void
toXYZ(float fx,
      float fy,
      float cx,
      float cy,
      int w,
      int h,
      float* Z,
      float* z,
      float* x,
      float* y)
{
    float* gpu_Z;
    float* gpu_z;
    float* gpu_x;
    float* gpu_y;

    size_t bytes = w * h * sizeof(float);

    // Allocate memory on GPU
    cudaMalloc(&gpu_Z, bytes);
    cudaMalloc(&gpu_z, bytes);
    cudaMalloc(&gpu_x, bytes);
    cudaMalloc(&gpu_y, bytes);
    // Copy Z - depth to GPU
    cudaMemcpy(gpu_Z, Z, bytes, cudaMemcpyHostToDevice);

    int threadsPerBlock = 32;
    dim3 blocks(threadsPerBlock, threadsPerBlock);
    dim3 grids(ceil((float)w / threadsPerBlock),
               ceil((float)h / threadsPerBlock));

    toXYZKernel<<<grids, blocks>>>(
            fx, fy, cx, cy, w, h, gpu_Z, gpu_z, gpu_x, gpu_y);

    cudaMemcpy(x, gpu_x, bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(y, gpu_y, bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(z, gpu_z, bytes, cudaMemcpyDeviceToHost);

    cudaFree(gpu_Z);
    cudaFree(gpu_z);
    cudaFree(gpu_x);
    cudaFree(gpu_y);
}

void
toXYZRGB(float fx,
         float fy,
         float cx,
         float cy,
         int w,
         int h,
         float* Z,
         unsigned char* rgb,
         pcl::PointXYZRGB* points)
{
    float* gpu_Z;
    pcl::PointXYZRGB* gpu_points;
    unsigned char* gpu_rgb;

    size_t bytes = w * h * sizeof(float);
    size_t rgb_t = 3 * w * h * sizeof(char);
    size_t gpu_t = bytes * 8;
    // Allocate memory on GPU
    cudaMalloc(&gpu_Z, bytes);
    cudaMalloc(&gpu_rgb, rgb_t);
    cudaMalloc(&gpu_points, gpu_t);

    // Copy Z - depth to GPU
    cudaMemcpy(gpu_Z, Z, bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_rgb, rgb, rgb_t, cudaMemcpyHostToDevice);

    int threadsPerBlock = 32;
    dim3 blocks(threadsPerBlock, threadsPerBlock);
    dim3 grids(ceil((float)w / threadsPerBlock),
               ceil((float)h / threadsPerBlock));

    toXYZRGBKernel<<<grids, blocks>>>(
            fx, fy, cx, cy, w, h, gpu_Z, gpu_rgb, gpu_points);

    cudaMemcpy(points, gpu_points, gpu_t, cudaMemcpyDeviceToHost);

    cudaFree(gpu_Z);
    cudaFree(gpu_rgb);
    cudaFree(gpu_points);
}
