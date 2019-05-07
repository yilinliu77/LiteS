#pragma once
#ifndef CCDATASTRUCTURE_H
#define CCDATASTRUCTURE_H

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "CBVHACCEL.h"
#include "CPointCloudMesh.h"

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

extern thrust::device_vector<float4> createDeviceVectorFloat4(int vNum);
extern thrust::device_vector<float3> createDeviceVectorFloat3(int vNum);
extern thrust::device_vector<float> createDeviceVectorFloat(int vNum);

extern __device__ bool d_intersect();

extern __device__ bool d_visible(BVHACCEL::LinearBVHNode* vNodes, Tri* vTriangles,
                                 float3 vCameraPos, float3 vVertexPosition,
                                 float margin = 0);

struct DBVHAccel {
	BVHACCEL::LinearBVHNode* vNodes;
	Tri* vTriangles;
	int numNodes;
	int numTriangles;
};

struct myFloat4 {
	float x, y, z, w;
};

}  // namespace CCDataStructure

#endif