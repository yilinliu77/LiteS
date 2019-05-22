#include <device_launch_parameters.h>
#include "kernel.h"

__global__ void updateRaysKernel(CCDataStructure::DBVHAccel* dBVH,
                                 CCDataStructure::Point* vPointCloud,
                                 int numPoints, glm::vec4* vRays,
                                 float3 vCameraPos) {
  int const bx = blockIdx.x;
  int const tx = threadIdx.x;

  int id = bx * blockDim.x + tx;
  if (id > numPoints) return;
  glm::vec3 pointPosition = vPointCloud[id].position;
  CCDataStructure::d_visible(dBVH->dBVHNodesPointer, dBVH->dTrianglesPointer,
                             CCDataStructure::float3ToGLM(vCameraPos),
                             pointPosition, 0.1f);

  vRays[id] = glm::vec4(1.f, 0.f, 1.f, 0.5f);
}

void updateObsRays(dim3 vGrid, dim3 vBlock, CCDataStructure::DBVHAccel* dBVH,
                   CCDataStructure::Point* vPointCloud, int numPoints,
                   glm::vec4* vRays, float3 vCameraPos) {
  updateRaysKernel<<<vGrid, vBlock>>>(dBVH, vPointCloud, numPoints, vRays,
                                      vCameraPos);
}