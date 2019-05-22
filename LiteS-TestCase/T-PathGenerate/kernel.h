#ifndef KERNAL_H
#define KERNAL_H

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "CCDataStructure.h"

extern void updateObsRays(dim3 vGrid, dim3 vBlock,
                          CCDataStructure::DBVHAccel* dBVH,
                          CCDataStructure::Point* vPointCloud, int numPoints,
                          glm::vec4* vRays, float3 vCameraPos);

#endif
