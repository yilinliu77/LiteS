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

thrust::device_vector<float4> CCDataStructure::createDeviceVectorFloat4(
    int vNum) {
  return thrust::device_vector<float4>(vNum);
}

thrust::device_vector<float3> CCDataStructure::createDeviceVectorFloat3(
    int vNum) {
  return thrust::device_vector<float3>(vNum);
}

thrust::device_vector<float> CCDataStructure::createDeviceVectorFloat(
    int vNum) {
  return thrust::device_vector<float>(vNum);
}

__device__ bool CCDataStructure::d_intersect(){ return false; }

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

  //Ray ray(vCameraPos, vVertexPosition - vCameraPos);
  //float current_t = normf(3,vVertexPosition - vCameraPos);

  //if (!nodes) return false;
  //bool hit = false;
  //glm::vec3 invDir(std::min(1 / ray.d.x, 99999999.0f),
  //                 std::min(1 / ray.d.y, 99999999.0f),
  //                 std::min(1 / ray.d.z, 99999999.0f));
  //int dirIsNeg[3] = {invDir[0] < 0, invDir[1] < 0, invDir[2] < 0};
  //// Follow ray through BVH nodes to find primitive intersections
  //int toVisitOffset = 0, currentNodeIndex = 0;
  //int nodesToVisit[64];
  //SurfaceInteraction isect;
  //while (true) {
  //  const LinearBVHNode* node = &nodes[currentNodeIndex];
  //  // Check ray against BVH node
  //  if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
  //    if (node->nObject > 0) {
  //      // Intersect ray with primitives in leaf BVH node
  //      for (int i = 0; i < node->nObject; ++i)
  //        if (orderedTriangles[node->objectOffset + i]->Intersect(ray, isect))
  //          hit = true;
  //      if (toVisitOffset == 0) break;
  //      currentNodeIndex = nodesToVisit[--toVisitOffset];
  //    } else {
  //      // Put far BVH node on _nodesToVisit_ stack, advance to near
  //      // node
  //      if (dirIsNeg[node->axis]) {
  //        nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
  //        currentNodeIndex = node->secondChildOffset;
  //      } else {
  //        nodesToVisit[toVisitOffset++] = node->secondChildOffset;
  //        currentNodeIndex = currentNodeIndex + 1;
  //      }
  //    }
  //  } else {
  //    if (toVisitOffset == 0) break;
  //    currentNodeIndex = nodesToVisit[--toVisitOffset];
  //  }
  //}
  //if (!hit) return false;
  //glm::vec3 hitPosition = isect.pHit;
  //if (current_t <= isect.t) {
  //  return true;
  //}
  return false;
}
