#include <device_launch_parameters.h>

#include "kernel.h"

#include <device_functions.h>

__global__ void updateRaysKernel(
    CCDataStructure::DBVHAccel* dBVH, CCDataStructure::Point* vPointCloud,
    int numPoints, CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
    float3 vCameraPos, float3 vCameraDirection) {
  const int bx = blockIdx.x;
  const int tx = threadIdx.x;

  int id = bx * blockDim.x + tx;
  if (id >= numPoints) return;
  int const stride = vRays.pitch / sizeof(glm::vec4);
  glm::vec3 pointPosition = vPointCloud[id].position;
  glm::vec3 cameraPosition = CCDataStructure::float3ToGLM(vCameraPos);
  glm::vec3 cameraDirection =
      glm::normalize(CCDataStructure::float3ToGLM(vCameraDirection));

  glm::vec3 point2Camera = glm::normalize(pointPosition - cameraPosition);
  float cosViewAngle = glm::dot(point2Camera, cameraDirection);
  // 0.7f ~ 90 degrees view fov
  if (cosViewAngle < 0.7f) return;

  bool visible =
      CCDataStructure::d_visible(dBVH, cameraPosition, pointPosition, 0.1f);
  if (visible) {
    unsigned int numRows = atomicAdd(vRays.numRowsPtr + id, 1u);
    glm::vec4* rays = vRays.dataPtr + id;
    rays[numRows * stride] =
        glm::vec4(point2Camera, glm::length(pointPosition - cameraPosition));
  }
}

void updateObsRays(dim3 vGrid, dim3 vBlock, cudaStream_t& vStream,
                   CCDataStructure::DBVHAccel* dBVH,
                   CCDataStructure::Point* vPointCloud, int numPoints,
                   CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
                   float3 vCameraPos, float3 vCameraDirection) {
  updateRaysKernel<<<vGrid, vBlock, 0, vStream>>>(
      dBVH, vPointCloud, numPoints, vRays,
                                      vCameraPos, vCameraDirection);
}

__device__ float calculateReconstrucbility(const glm::vec4& vertex2Camera1,
                                           const glm::vec4& vertex2Camera2,
                                           glm::vec3 vertexNormal) {
  const float DMAX = 100.f;

  vertexNormal = glm::normalize(vertexNormal);
  glm::vec3 v2c1 = glm::vec3(vertex2Camera1);
  glm::vec3 v2c2 = glm::vec3(vertex2Camera2);

  float camera1Distance = vertex2Camera1[3];
  float camera2Distance = vertex2Camera2[3];
  float maxDistance = glm::max(camera1Distance, camera2Distance);

  if (maxDistance > DMAX) return 0.f;

  float viewAngleCos = glm::clamp(glm::dot(v2c1, v2c2), 0.f, 0.9999f);
  float viewAngle = glm::acos(viewAngleCos);

  // w1 = 1 / ( 1 + exp( -k1 * ( alpha - alpha1 ) ) )
  float tempw1 = (viewAngle - glm::pi<float>() / 16.0f);
  float w1 = 1 / (1 + (glm::exp(-32 * tempw1)));
  
  // w2 = min( max(d1, d2) / dmax, 1 )
  float w2 = glm::min(maxDistance / DMAX, 1.f);

  // w3 =1 - 1 / ( 1 + exp( -k3 * ( alpha - alpha3 ) ) )
  float w3 = 1 - 1 / (1 + glm::exp(-8 * (viewAngle - glm::pi<float>() / 4.0f)));

  if (glm::distance(v2c1, v2c2) < 0.01f) return 0.f;
  glm::vec3 viewPlaneNormal = glm::normalize(glm::cross(v2c1, v2c2));
  float theta1 = glm::max(glm::dot(v2c1, vertexNormal), 0.f);
  float theta2 = glm::max(glm::dot(v2c2, vertexNormal), 0.f);
  float theta = (theta1 + theta2) / 2;

  float score = w1 * w2 * w3 * theta;
  assert(score >= 0.f);
  return score;
}

__global__ void evaluateReconstrucbilityKernel(
    int numPoints, CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
    float* vReconScorePointer, CCDataStructure::Point* vPointPtr) {
  const int bx = blockIdx.x;
  const int tx = threadIdx.x;

  int id = bx * blockDim.x + tx;
  if (id >= numPoints) return;

  float reconstrucbility = 0.f;
  unsigned int numRows = *(vRays.numRowsPtr + id);
  int const stride = vRays.pitch / sizeof(glm::vec4);

  glm::vec4* rays = vRays.dataPtr + id;
  for (int i = 0; i < numRows; ++i) {
    for (int j = i + 1; j < numRows; ++j) {
      // glm::vec4 rayProp = vRays.dataPtr + id;
      glm::vec4 vertex2Camera1 = rays[i + stride];
      glm::vec4 vertex2Camera2 = rays[j + stride];
      glm::vec3 normal = vPointPtr[id].normal;
      reconstrucbility +=
          calculateReconstrucbility(vertex2Camera1, vertex2Camera2, normal);
    }
  }
  vReconScorePointer[id] = reconstrucbility;
}

void evaluateReconstrucbility(
    dim3 vGrid, dim3 vBlock, cudaStream_t vStream, int numPoints,
    CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
    float* vReconScorePointer, CCDataStructure::Point* vPointPtr) {
  evaluateReconstrucbilityKernel<<<vGrid, vBlock, 0, vStream>>>(
      numPoints, vRays, vReconScorePointer, vPointPtr);
}