#ifndef KERNAL_H
#define KERNAL_H

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "CCDataStructure.h"
#include "CCVectorArray.cuh"
extern void updateObsRays(dim3 vGrid, dim3 vBlock,
                          cudaStream_t& vStream,
                          CCDataStructure::DBVHAccel* dBVH,
                          CCDataStructure::Point* vPointCloud, int numPoints,
                          CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
                          float3 vCameraPos, float3 vCameraDirection);

extern void evaluateReconstrucbility(
    dim3 vGrid, dim3 vBlock, cudaStream_t vStream, int numPoints,
    CCDataStructure::CCVectorArray<glm::vec4>::Data vRays,
    float* vReconScorePointer, CCDataStructure::Point* vPointPtr);

#endif
