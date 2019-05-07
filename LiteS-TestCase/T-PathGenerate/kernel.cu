#include <device_launch_parameters.h>
#include "kernel.h"

__global__ void updateRaysKernel(CCDataStructure::DBVHAccel* dBVH,
                                 CCDataStructure::Point* vPointCloud,
                                 int numPoints, float4* vRays,
                                 float3 vCameraPos) {
  int const bx = blockIdx.x;
  int const tx = threadIdx.x;

  int id = bx * blockDim.x + tx;
  if (id > numPoints) return;
  float3 pointPosition = vPointCloud[id].position;
  CCDataStructure::d_visible(dBVH->vNodes, dBVH->vTriangles, vCameraPos,
                             pointPosition, 0.1f);

  vRays[id] = make_float4(1.f, 0.f, 1.f, 0.5f);
}

__global__ void updateObsRays(dim3 vGrid, dim3 vBlock,
                              CCDataStructure::DBVHAccel* dBVH,
                              CCDataStructure::Point* vPointCloud,
                              int numPoints, float4* vRays, float3 vCameraPos) {
  updateRaysKernel<<<vGrid, vBlock>>>(dBVH, vPointCloud, numPoints, vRays,
                                      vCameraPos);
}