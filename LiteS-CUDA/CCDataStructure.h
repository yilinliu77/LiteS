#pragma once
#ifndef CCDATASTRUCTURE_H
#define CCDATASTRUCTURE_H

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "CBVHACCEL.h"
#include "CPointCloudMesh.h"
#define CUDACHECKERROR(ans) \
  { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) exit(code);
  }
}
namespace CCDataStructure {

inline unsigned int divup(unsigned int a, unsigned int b) {
  return a / b + (a % b != 0);
}

struct Point {
  float3 position;
  float3 normal;
};

extern thrust::device_vector<CCDataStructure::Point> createDevicePointCloud(
    CMesh* vPointCloud);

extern thrust::device_vector<glm::vec4> createDeviceVectorGLM4(int vNum);
extern thrust::device_vector<float3> createDeviceVectorGLM3(int vNum);
extern thrust::device_vector<float> createDeviceVectorFloat(int vNum);

extern __device__ bool d_intersect();

extern __device__ bool d_visible(BVHACCEL::LinearBVHNode* vNodes,
                                 Tri* vTriangles, float3 vCameraPos,
                                 float3 vVertexPosition, float margin = 0);

struct DBVHAccel {
	DBVHAccel(const BVHACCEL::BVHAccel* bvhTree);

  thrust::device_vector<Tri>* dTriangles;
  thrust::device_vector<BVHACCEL::LinearBVHNode>* dBVHNodes;

  BVHACCEL::LinearBVHNode* dBVHNodesPointer;
  Tri* dTrianglesPointer;
  int numNodes;
  int numTriangles;
};

CCDataStructure::DBVHAccel* createDBVHAccel(const BVHACCEL::BVHAccel* bvhTree) {
  DBVHAccel* hDBVHAccel = new DBVHAccel(bvhTree);

  DBVHAccel* dDBVHAccel;
  CUDACHECKERROR(cudaMalloc(&dDBVHAccel, sizeof(DBVHAccel)));
  CUDACHECKERROR(cudaMemcpy(dDBVHAccel, hDBVHAccel, sizeof(DBVHAccel),
                            cudaMemcpyHostToDevice));

  delete hDBVHAccel;
  return dDBVHAccel;
}

struct myFloat4 {
  float x, y, z, w;
};

}  // namespace CCDataStructure

#endif