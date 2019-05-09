#include <thrust/copy.h>
#include "CCDataStructure.h"

thrust::device_vector<CCDataStructure::Point>
CCDataStructure::createDevicePointCloud(CMesh* vPointCloud) {
  thrust::device_vector<CCDataStructure::Point> dPoints;

  for (const auto& item : vPointCloud->vertices) {
    CCDataStructure::Point p;
    p.position.x = item.Position[0];
    p.position.y = item.Position[1];
    p.position.z = item.Position[2];
    p.normal.x = item.Normal[0];
    p.normal.y = item.Normal[1];
    p.normal.z = item.Normal[2];
    dPoints.push_back(p);
  }

  return dPoints;
}

thrust::device_vector<glm::vec4> CCDataStructure::createDeviceVectorGLM4(
    int vNum) {
  return thrust::device_vector<glm::vec4>(vNum);
}

thrust::device_vector<float3> CCDataStructure::createDeviceVectorGLM3(
    int vNum) {
  return thrust::device_vector<float3>(vNum);
}

thrust::device_vector<float> CCDataStructure::createDeviceVectorFloat(
    int vNum) {
  return thrust::device_vector<float>(vNum);
}

__device__ bool CCDataStructure::d_intersect() { return false; }

CCDataStructure::DBVHAccel* moveTreeToDevice(
    const BVHACCEL::BVHAccel* bvhTree) {
  CCDataStructure::DBVHAccel* dBVHAccel = new CCDataStructure::DBVHAccel();
  thrust::device_vector<Tri>* dTriangles = new thrust::device_vector<Tri>();
  thrust::device_vector<BVHACCEL::LinearBVHNode>* dBVHNodes =
      new thrust::device_vector<BVHACCEL::LinearBVHNode>();

  thrust::copy(bvhTree->getOrderedTriangles().begin(),
               bvhTree->getOrderedTriangles().end(), dTriangles->begin());
  dBVHAccel->dTrianglesPointer = thrust::raw_pointer_cast(&(*dTriangles)[0]);
  // dBVHAccel->numTriangles = bvhTree->getOrderedTriangles().size();

  // dBVHAccel->numNodes = bvhTree->totalLinearNodes;
  // thrust::copy(bvhTree->getLinearNodes().begin(),
  //             bvhTree->getLinearNodes().end(),
  //             dBVHNodes->begin());
  // dBVHAccel->dBVHNodesPointer = thrust::raw_pointer_cast(&(*dBVHNodes)[0]);

  return dBVHAccel;
}

CCDataStructure::DBVHAccel* CCDataStructure::createDBVHAccel(
    const BVHACCEL::BVHAccel* bvhTree) {
  CCDataStructure::DBVHAccel* hDBVHAccel = new CCDataStructure::DBVHAccel();

  hDBVHAccel->numTriangles = bvhTree->orderedTriangles.size();
  hDBVHAccel->numNodes = bvhTree->totalLinearNodes;
  CUDACHECKERROR(
      cudaMalloc((void**)&(hDBVHAccel->dBVHNodesPointer),
                 sizeof(BVHACCEL::LinearBVHNode) * bvhTree->totalLinearNodes));
  CUDACHECKERROR(cudaMalloc((void**)&hDBVHAccel->dTrianglesPointer,
                            sizeof(Tri) * bvhTree->orderedTriangles.size()));

  CUDACHECKERROR(
      cudaMemcpy(hDBVHAccel->dBVHNodesPointer, &bvhTree->nodes[0],
                 sizeof(BVHACCEL::LinearBVHNode) * bvhTree->totalLinearNodes,
                 cudaMemcpyHostToDevice));
  CUDACHECKERROR(cudaMemcpy(
      hDBVHAccel->dTrianglesPointer, &bvhTree->orderedTriangles[0],
      sizeof(Tri) * bvhTree->orderedTriangles.size(), cudaMemcpyHostToDevice));

  CCDataStructure::DBVHAccel* dDBVHAccel;
  CUDACHECKERROR(cudaMalloc((void**)&dDBVHAccel, sizeof(DBVHAccel)));
  CUDACHECKERROR(cudaMemcpy(dDBVHAccel, hDBVHAccel,
                            sizeof(CCDataStructure::DBVHAccel),
                            cudaMemcpyHostToDevice));

  return dDBVHAccel;
}

__device__ bool CCDataStructure::d_visible(BVHACCEL::LinearBVHNode* vNodes,
                                           Tri* vTriangles, float3 vCameraPos,
                                           float3 vVertexPosition,
                                           float margin) {
  if (margin > 0) {
    vVertexPosition.z += margin;
    if (vCameraPos.x > 0)
      vVertexPosition.x += margin;
    else
      vVertexPosition.x += -margin;
    if (vCameraPos.y > 0)
      vVertexPosition.y += margin;
    else
      vVertexPosition.y += -margin;
  }
  return false;
}