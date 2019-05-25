#include "CCDataStructure.h"


namespace CCDataStructure {
void createDevicePointCloud(CMesh* vPointCloud,
                            thrust::device_vector<Point>& vDPoints) {
  thrust::host_vector<Point> hPoints;

  for (const auto& item : vPointCloud->vertices) {
    Point p;
    p.position.x = item.Position[0];
    p.position.y = item.Position[1];
    p.position.z = item.Position[2];
    p.normal.x = item.Normal[0];
    p.normal.y = item.Normal[1];
    p.normal.z = item.Normal[2];
    hPoints.push_back(p);
  }

  vDPoints = hPoints;
}

thrust::device_vector<glm::vec4> createDeviceVectorGLM4(int vNum) {
  return thrust::device_vector<glm::vec4>(vNum);
}

thrust::device_vector<float3> createDeviceVectorGLM3(int vNum) {
  return thrust::device_vector<float3>(vNum);
}

thrust::device_vector<float> createDeviceVectorFloat(int vNum) {
  return thrust::device_vector<float>(vNum);
}

DBVHAccel* createDBVHAccel(const ACCEL::BVHAccel* bvhTree) {
  DBVHAccel* hDBVHAccel = new DBVHAccel();

  hDBVHAccel->numTriangles = bvhTree->orderedTriangles.size();
  hDBVHAccel->numNodes = bvhTree->totalLinearNodes;
  CUDACHECKERROR(
      cudaMalloc((void**)&(hDBVHAccel->dBVHNodesPointer),
                 sizeof(ACCEL::LinearBVHNode) * bvhTree->totalLinearNodes));
  CUDACHECKERROR(cudaMalloc((void**)&hDBVHAccel->dTrianglesPointer,
                            sizeof(Tri) * bvhTree->orderedTriangles.size()));

  CUDACHECKERROR(
      cudaMemcpy(hDBVHAccel->dBVHNodesPointer, &bvhTree->nodes[0],
                 sizeof(ACCEL::LinearBVHNode) * bvhTree->totalLinearNodes,
                 cudaMemcpyHostToDevice));
  CUDACHECKERROR(cudaMemcpy(
      hDBVHAccel->dTrianglesPointer, &bvhTree->orderedTriangles[0],
      sizeof(Tri) * bvhTree->orderedTriangles.size(), cudaMemcpyHostToDevice));

  DBVHAccel* dDBVHAccel;
  CUDACHECKERROR(cudaMalloc((void**)&dDBVHAccel, sizeof(DBVHAccel)));
  CUDACHECKERROR(cudaMemcpy(dDBVHAccel, hDBVHAccel, sizeof(DBVHAccel),
                            cudaMemcpyHostToDevice));

  return dDBVHAccel;
}

__device__ bool d_intersect(const DBVHAccel* vBVHPointer, Ray& ray,
                            SurfaceInteraction* isect) {
  const ACCEL::LinearBVHNode* nodes = vBVHPointer->dBVHNodesPointer;
  const Tri* tris = vBVHPointer->dTrianglesPointer;
  const int numNodes = vBVHPointer->numNodes;
  const int numTriangles = vBVHPointer->numTriangles;
  bool hit = false;
  glm::vec3 invDir(glm::min(1 / ray.d.x, 99999.0f),
                   glm::min(1 / ray.d.y, 99999.0f),
                   glm::min(1 / ray.d.z, 99999.0f));
  int dirIsNeg[3] = {invDir[0] < 0, invDir[1] < 0, invDir[2] < 0};
  // Follow ray through BVH nodes to find primitive intersections
  int toVisitOffset = 0, currentNodeIndex = 0;
  int nodesToVisit[6400];

  while (true) {
    if (currentNodeIndex >= numNodes) {
      printf("Wrong Node %d\n", currentNodeIndex);
      return false;
    }
    const ACCEL::LinearBVHNode* node = &nodes[currentNodeIndex];
    // Check ray against BVH node
    float a, b;
    if (node->bounds.Intersect(ray, &a, &b)) {
      if (node->nObject > 0) {
        // Intersect ray with primitives in leaf BVH node
        for (int i = 0; i < node->nObject; ++i) {
          if (node->objectOffset + i >= numTriangles) {
            printf("Wrong tri %d\n", node->objectOffset + i);
            return false;
          }
          if (tris[node->objectOffset + i].Intersect(ray, isect)) hit = true;

        }
        if (toVisitOffset == 0) break;
        currentNodeIndex = nodesToVisit[--toVisitOffset];
      } else {
        if (dirIsNeg[node->axis]) {
          nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
          currentNodeIndex = node->secondChildOffset;
        } else {
          nodesToVisit[toVisitOffset++] = node->secondChildOffset;
          currentNodeIndex = currentNodeIndex + 1;
        }
      }
    } else {
      if (toVisitOffset == 0) break;
      currentNodeIndex = nodesToVisit[--toVisitOffset];
    }
  }
  return hit;
}

__device__ bool d_visible(const DBVHAccel* vBVHPointer,
                          const glm::vec3 vCameraPos,
                          const glm::vec3 vVertexPosition,
                          const float margin) {
  Ray ray(vCameraPos, vVertexPosition - vCameraPos);
  float current_t = glm::length(vVertexPosition - vCameraPos);
  SurfaceInteraction isect;
  if (!d_intersect(vBVHPointer, ray, &isect)) return false;
  if (current_t <= isect.t) {
    return true;
  }
  return false;
}

}  // namespace CCDataStructure