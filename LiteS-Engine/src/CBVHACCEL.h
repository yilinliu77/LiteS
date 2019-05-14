#ifndef BVHTREE_HEADER
#define BVHTREE_HEADER

#include <glm/common.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include "CMesh.h"

const int MAX_TRIANGLES_IN_NODE = 255;

namespace BVHACCEL {
enum SplitMethod { SAH, HLBVH, Middle, EqualCounts };

struct BVHBuildNode {
  Bounds3f bounds;
  BVHBuildNode *leftChild, *rightChild;
  int splitAxis;
  int index;
  int nObjects;
  void initLeaf(const Bounds3f& b, const int i, const int n) {
    nObjects = n;
    index = i;
    bounds = b;
    leftChild = NULL;
    rightChild = NULL;
  }

  void initInterior(int axis, BVHBuildNode* c0, BVHBuildNode* c1) {
    index = -1;
    splitAxis = axis;
    leftChild = c0;
    rightChild = c1;
    bounds = c0->bounds.unionBounds(c1->bounds);
    nObjects = 0;
  }
};

struct LinearBVHNode {
  Bounds3f bounds;
  float boundVertices[24];
  int objectOffset = -1;
  int secondChildOffset;
  int parentOffset;  // LAST
  uint16_t nObject;
  uint8_t axis;
};

class BVHAccel {
 public:
  int totalLinearNodes;
  std::vector<Tri> totalTriangles;
  std::vector<Tri> orderedTriangles;
  std::vector<LinearBVHNode> nodes;

  BVHAccel(const CMesh* p) : splitMethod(SAH) {
    if (p == nullptr) return;


    for (int i = 0; i < p->indices.size() / 3; ++i) {
      Tri t(p->vertices[p->indices[i * 3]],
            p->vertices[p->indices[i * 3 + 1]],
            p->vertices[p->indices[i * 3 + 2]]);
      totalTriangles.push_back(t);
    }


    totalLinearNodes = 0;

    // init
    BVHBuildNode* root;
    int totalNodes = 0;
    root = recursiveBuildTree(totalTriangles, 0, (int)totalTriangles.size(),
                              &totalNodes, orderedTriangles);

    // compact the tree
    nodes.resize(totalNodes);
    int offset = 0;
    flattenBVHTree(root, &offset, -1);
  }
  ~BVHAccel() {}
  const std::vector<LinearBVHNode>& getLinearNodes() const { return nodes; }
  const std::vector<Tri>& getOrderedTriangles() const {
    return orderedTriangles;
  }

  bool Intersect(Ray& ray, SurfaceInteraction* isect) const {
    if (nodes.empty()) return false;
    bool hit = false;
    glm::vec3 invDir(std::min(1 / ray.d.x, 99999999.0f),
                     std::min(1 / ray.d.y, 99999999.0f),
                     std::min(1 / ray.d.z, 99999999.0f));
    int dirIsNeg[3] = {invDir[0] < 0, invDir[1] < 0, invDir[2] < 0};
    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];
    while (true) {
      const LinearBVHNode* node = &nodes[currentNodeIndex];
      // Check ray against BVH node
      //if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
      float a, b;
      if (node->bounds.Intersect(ray,&a,&b)){
        if (node->nObject > 0) {
          // Intersect ray with primitives in leaf BVH node
          for (int i = 0; i < node->nObject; ++i)
            if (orderedTriangles[node->objectOffset + i].Intersect(ray, isect))
              hit = true;
          if (toVisitOffset == 0) break;
          currentNodeIndex = nodesToVisit[--toVisitOffset];
        } else {
          // Put far BVH node on _nodesToVisit_ stack, advance to near
          // node
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

  std::pair<float, glm::vec3> KNearest(const glm::vec3 vPoint) {
    glm::vec3 closetPoint;
    float closetLength = 99999999.f;
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
      const LinearBVHNode* node = &nodes[currentNodeIndex];
      if (node->nObject > 0) {
        for (int i = 0; i < node->nObject; ++i) {
          float length1 = glm::length(
              orderedTriangles[node->objectOffset + i].v1.Position - vPoint);
          float length2 = glm::length(
              orderedTriangles[node->objectOffset + i].v2.Position - vPoint);
          float length3 = glm::length(
              orderedTriangles[node->objectOffset + i].v3.Position - vPoint);
          if (length1 < closetLength) {
            closetLength = length1;
            closetPoint = orderedTriangles[node->objectOffset + i].v1.Position;
          }
          if (length2 < closetLength) {
            closetLength = length2;
            closetPoint = orderedTriangles[node->objectOffset + i].v2.Position;
          }
          if (length3 < closetLength) {
            closetLength = length3;
            closetPoint = orderedTriangles[node->objectOffset + i].v3.Position;
          }
        }
        break;
      } else {
        if (glm::length(nodes[currentNodeIndex + 1].bounds.getCentroid() -
                        vPoint) <
            glm::length(nodes[node->secondChildOffset].bounds.getCentroid() -
                        vPoint)) {
          currentNodeIndex = currentNodeIndex + 1;
        } else {
          currentNodeIndex = node->secondChildOffset;
        }
      }
    }
    assert(closetLength < 99999999);
    return {closetLength, closetPoint};
  }

  bool Visible(glm::vec3 vOberveVertex, glm::vec3 vVertex,
                         float margin = 0) {
    if (margin > 0) {
      vVertex.z += margin;
      if (vOberveVertex.x > 0)
        vVertex.x += margin;
      else
        vVertex.x += -margin;
      if (vOberveVertex.y > 0)
        vVertex.y += margin;
      else
        vVertex.y += -margin;
    }

    Ray ray(vOberveVertex, vVertex - vOberveVertex);
    float current_t = glm::length(vVertex - vOberveVertex);
    SurfaceInteraction isect;
    if (!Intersect(ray, &isect)) return false;
    glm::vec3 hitPosition = isect.pHit;
    if (current_t <= isect.t) {
      return true;
    }
    return false;
  }

  bool strongVisible(glm::vec3 vCameraPos,
                               glm::vec3 vCameraOrientation,
                               glm::vec3 vSamplePosition, float vDMAX) {
    glm::vec3 sampleToCamera = vCameraPos - vSamplePosition;
    // Inside the frustum
    // 0.6f ~ cos(78.0f / 2 / 180.0f * pi)
    float viewAngleCos = glm::dot(-glm::normalize(sampleToCamera),
                                  glm::normalize(vCameraOrientation));
    if (viewAngleCos < 0.77) return false;

    if (glm::length(vSamplePosition - vCameraPos) > vDMAX) return false;

    // Not occluded
    if (!this->Visible(vCameraPos, vSamplePosition, 1.0f)) return false;

    return true;
  }

 private:
  const SplitMethod splitMethod;

  BVHBuildNode* recursiveBuildTree(std::vector<Tri>& p, int start, int end,
                                   int* totalNodes,
                                   std::vector<Tri>& orderedObjects) {
    // return value,the earliest value -> root
    BVHBuildNode* node = new BVHBuildNode();
    (*totalNodes)++;
    int nObjects = end - start;
    //      int nTris = end - start;
    // for (int i = start; i < end; i++) {
    //	nTris += p[i]->vertices.size();
    //}

    // judge the status of recur
    if (nObjects == 1) {
      node->initLeaf(p[start].bounds, (int)orderedTriangles.size(), nObjects);
      for (int i = start; i < end; ++i) orderedTriangles.push_back(p[i]);
      return node;
    } else {
      // compute all the bounds now between start and end
      Bounds3f bounds = p[start].bounds;
      for (int i = start + 1; i < end; i++)
        bounds = bounds.unionBounds(p[i].bounds);
      // detect if the bounds have zero volume
      // TODO
      // if (bounds.pMax==bounds.pMin)
      //{

      //}

      // select which axis should be dimension
      int dim = bounds.MaximumExtent();

      // partition
      int mid = -1;
      // when the number of object is less than 4,use equal set strategy
      if (nObjects <= 4) {
        mid = (start + end) / 2;
        std::nth_element(
            &p[start], &p[mid], &p[end - 1] + 1, [dim](Tri a, Tri b) {
              return a.bounds.getCentroid()[dim] < b.bounds.getCentroid()[dim];
            });
      } else {
        const int nBuckets = 12;
        struct BucketInfo {
          Bounds3f bounds;
          int count = 0;
        };
        BucketInfo buckets[nBuckets];
        for (int i = start; i < end; ++i) {
          int b = static_cast<int>(
              nBuckets * bounds.Offset(p[i].bounds.getCentroid())[dim]);
          if (b == nBuckets) b -= 1;
          ++buckets[b].count;
          buckets[b].bounds = buckets->bounds.unionBounds(p[i].bounds);
        }

        // compute the cost
        float cost[nBuckets - 1];
        for (int i = 0; i < nBuckets - 1; ++i) {
          Bounds3f b0, b1;
          int count0 = 0, count1 = 0;
          for (int j = 0; j <= i; j++) {
            b0 = b0.unionBounds(buckets[j].bounds);
            count0 += buckets[j].count;
          }
          for (int j = i + 1; j < nBuckets - 1; j++) {
            b1 = b1.unionBounds(buckets[j].bounds);
            count1 += buckets[j].count;
          }
          cost[i] =
              0.125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) /
                           bounds.SurfaceArea();
        }
        // find the minimizes SAH metric
        float minCost = cost[0];
        int minCostSplitBucket = 0;
        for (int i = 1; i < nBuckets - 1; i++) {
          if (cost[i] < minCost) {
            minCost = cost[i];
            minCostSplitBucket = i;
          }
        }
        // partition
        float leafCost = static_cast<float>(nObjects);
        if (nObjects > MAX_TRIANGLES_IN_NODE || minCost < leafCost) {
          Tri* pMid = std::partition(&p[start], &p[end - 1] + 1, [=](Tri pi) {
            int b = static_cast<int>(
                nBuckets * bounds.Offset(pi.bounds.getCentroid())[dim]);
            if (b == nBuckets) b -= 1;
            return b <= minCostSplitBucket;
          });
          mid = static_cast<int>(pMid - &p[0]);
          if (mid == start || mid == end) {  // judge if buckets devision failed
            mid = (start + end) / 2;
            std::nth_element(&p[start], &p[mid], &p[end - 1] + 1,
                             [dim](Tri a, Tri b) {
                               return a.bounds.getCentroid()[dim] <
                                      b.bounds.getCentroid()[dim];
                             });
          }
        } else {
          node->initLeaf(bounds, (int)orderedTriangles.size(), nObjects);
          for (int i = start; i < end; ++i) orderedTriangles.push_back(p[i]);
          return node;
        }
      }
      node->initInterior(
          dim, recursiveBuildTree(p, start, mid, totalNodes, orderedTriangles),
          recursiveBuildTree(p, mid, end, totalNodes, orderedTriangles));
    }
    return node;
  }

  int flattenBVHTree(BVHBuildNode* node, int* offset, int parentOffset) {
    totalLinearNodes++;
    LinearBVHNode* linearBVHNode = &nodes[*offset];
    linearBVHNode->parentOffset = parentOffset;
    linearBVHNode->bounds = node->bounds;
    int myOffset = (*offset)++;
    if (node->leftChild != NULL && node->rightChild != NULL) {
      linearBVHNode->axis = node->splitAxis;
      flattenBVHTree(node->leftChild, offset, myOffset);
      linearBVHNode->secondChildOffset =
          flattenBVHTree(node->rightChild, offset, myOffset);
      linearBVHNode->nObject = 0;
      linearBVHNode->objectOffset = -1;
    } else {
      linearBVHNode->objectOffset = node->index;
      linearBVHNode->nObject = node->nObjects;
    }
    float x =
        (linearBVHNode->bounds.pMax[0] - linearBVHNode->bounds.pMin[0]) / 2;
    float y =
        (linearBVHNode->bounds.pMax[1] - linearBVHNode->bounds.pMin[1]) / 2;
    float z =
        (linearBVHNode->bounds.pMax[2] - linearBVHNode->bounds.pMin[2]) / 2;
    glm::vec3 boundVerticesVector[] = {
        linearBVHNode->bounds.getCentroid() + glm::vec3(x, y, z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(x, -y, z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(x, -y, -z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(x, y, -z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(-x, -y, -z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(-x, -y, z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(-x, y, z),
        linearBVHNode->bounds.getCentroid() + glm::vec3(-x, y, -z),
    };
    for (int i = 0; i < 8; ++i) {
      linearBVHNode->boundVertices[3 * i] = boundVerticesVector[i][0];
      linearBVHNode->boundVertices[3 * i + 1] = boundVerticesVector[i][1];
      linearBVHNode->boundVertices[3 * i + 2] = boundVerticesVector[i][2];
    }

    return myOffset;
  }
};
}  // namespace BVHACCEL

#endif  // BVHTREE_HEADER