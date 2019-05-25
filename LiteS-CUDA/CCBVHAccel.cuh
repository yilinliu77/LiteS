#ifndef CCBVHACCEL_H
#define CCBVHACCEL_H


#include <CBVHACCEL.h>
#include "CCDataStructure.h"
#include "CCPrimitive.cuh"

namespace CCDataStructure {

class BVHAccel {
  __device__ __host__ BVHAccel(const ACCEL::BVHAccel* bvhTree) {
    size_t numTriangles = bvhTree->orderedTriangles.size();
    size_t numNodes = bvhTree->totalLinearNodes;
    CUDACHECKERROR(
        cudaMalloc((void**)&(this->dBVHNodesPointer),
                   sizeof(ACCEL::LinearBVHNode) * bvhTree->totalLinearNodes));
    CUDACHECKERROR(cudaMalloc((void**)&this->dTrianglesPointer,
                              sizeof(Tri) * bvhTree->orderedTriangles.size()));

    CUDACHECKERROR(
        cudaMemcpy(hDBVHAccel->dBVHNodesPointer, &bvhTree->nodes[0],
                   sizeof(ACCEL::LinearBVHNode) * bvhTree->totalLinearNodes,
                   cudaMemcpyHostToDevice));
    CUDACHECKERROR(cudaMemcpy(hDBVHAccel->dTrianglesPointer,
                              &bvhTree->orderedTriangles[0],
                              sizeof(Tri) * bvhTree->orderedTriangles.size(),
                              cudaMemcpyHostToDevice));

    DBVHAccel* dDBVHAccel;
    CUDACHECKERROR(cudaMalloc((void**)&dDBVHAccel, sizeof(DBVHAccel)));
    CUDACHECKERROR(cudaMemcpy(dDBVHAccel, hDBVHAccel, sizeof(DBVHAccel),
                              cudaMemcpyHostToDevice));
  }

 public:
  ACCEL::LinearBVHNode* dBVHNodesPointer;
  Tri* dTrianglesPointer;
  int numNodes;
  int numTriangles;
};

}  // namespace CCDataStructure

#endif  // !CCBVHACCEL_H
