#ifndef BVHTREE_HEADER
#define BVHTREE_HEADER

#include <glm/glm.hpp>
#include <iostream>
#include <glm/common.hpp>
#include "CMesh.h"

const int MAX_TRIANGLES_IN_NODE = 255;

enum SplitMethod { SAH,
    HLBVH,
    Middle,
    EqualCounts };

struct BVHBuildNode {
    Bounds3f bounds;
    BVHBuildNode *leftChild, *rightChild;
    int splitAxis;
    int index;
    int nObjects;
    void initLeaf(const Bounds3f& b, const int i, const int n)
    {
        nObjects = n;
        index = i;
        bounds = b;
        leftChild = NULL;
        rightChild = NULL;
    }

    void initInterior(int axis, BVHBuildNode* c0, BVHBuildNode* c1)
    {
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
    int parentOffset; // LAST
    uint16_t nObject;
    uint8_t axis;
};

class BVHAccel {
public:
    int totalLinearNodes;
	std::vector<Tri> totalTriangles;

    BVHAccel(const std::vector<CMesh*>& p):splitMethod(SAH){
        if (p.size() == 0)
            return;

		for (auto * mesh: p)
		{
			for (int i = 0; i < mesh->indices.size() / 3; ++i) {
				Tri t(mesh->vertices[mesh->indices[i * 3]]
					, mesh->vertices[mesh->indices[i * 3 + 1]]
					, mesh->vertices[mesh->indices[i * 3 + 2]]);
				totalTriangles.push_back(t);
			}
		}

        totalLinearNodes = 0;

        // init
        BVHBuildNode* root;
        std::vector<CMesh*> pClone(p);
        int totalNodes = 0;
        root = recursiveBuildTree(totalTriangles, 0, totalTriangles.size(), &totalNodes, orderedTriangles);

        // compact the tree
        nodes = (LinearBVHNode*)malloc(sizeof(LinearBVHNode) * totalNodes);
        int offset = 0;
        flattenBVHTree(root, &offset, -1);
    }
    ~BVHAccel() {}
    LinearBVHNode* getLinearNodes() { return nodes; }
    std::vector<Tri*> getOrderedTriangles() { return orderedTriangles; }

	bool BVHAccel::Intersect(Ray &ray, SurfaceInteraction *isect) const {
		if (!nodes) return false;
		bool hit = false;
		glm::vec3 invDir(std::min(1 / ray.d.x, 99999999.0f)
			, std::min(1 / ray.d.y, 99999999.0f)
			, std::min(1 / ray.d.z, 99999999.0f));
		int dirIsNeg[3] = { invDir[0] < 0, invDir[1] < 0, invDir[2] < 0 };
		// Follow ray through BVH nodes to find primitive intersections
		int toVisitOffset = 0, currentNodeIndex = 0;
		int nodesToVisit[64];
		while (true) {
			const LinearBVHNode *node = &nodes[currentNodeIndex];
			// Check ray against BVH node
			if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
				if (node->nObject > 0) {
					// Intersect ray with primitives in leaf BVH node
					for (int i = 0; i < node->nObject; ++i)
						if (orderedTriangles[node->objectOffset + i]->Intersect(
							ray, isect))
							hit = true;
					if (toVisitOffset == 0) break;
					currentNodeIndex = nodesToVisit[--toVisitOffset];
				}
				else {
					// Put far BVH node on _nodesToVisit_ stack, advance to near
					// node
					if (dirIsNeg[node->axis]) {
						nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
						currentNodeIndex = node->secondChildOffset;
					}
					else {
						nodesToVisit[toVisitOffset++] = node->secondChildOffset;
						currentNodeIndex = currentNodeIndex + 1;
					}
				}
			}
			else {
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
		}
		return hit;
	}

	std::pair<float,glm::vec3> BVHAccel::KNearest(const glm::vec3 vPoint) {
		glm::vec3 closetPoint;
		float closetLength = 99999999;
		int toVisitOffset = 0, currentNodeIndex = 0;
		int nodesToVisit[64];
		while (true) {
			const LinearBVHNode *node = &nodes[currentNodeIndex];
			// Check ray against BVH node
			if (node->bounds.inside(vPoint)) {
				if (node->nObject > 0) {
					for (int i = 0; i < node->nObject; ++i) {
						float length1 = glm::length(orderedTriangles[node->objectOffset + i]->v1.Position - vPoint);
						float length2 = glm::length(orderedTriangles[node->objectOffset + i]->v2.Position - vPoint);
						float length3 = glm::length(orderedTriangles[node->objectOffset + i]->v3.Position - vPoint);
						if (length1 < closetLength)
						{
							closetLength = length1;
							closetPoint = orderedTriangles[node->objectOffset + i]->v1.Position;
						}
						if (length2 < closetLength)
						{
							closetLength = length2;
							closetPoint = orderedTriangles[node->objectOffset + i]->v2.Position;
						}
						if (length3 < closetLength)
						{
							closetLength = length3;
							closetPoint = orderedTriangles[node->objectOffset + i]->v3.Position;
						}
					}
					if (toVisitOffset == 0) break;
					currentNodeIndex = nodesToVisit[--toVisitOffset];
				}
				else {
					nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
					currentNodeIndex = node->secondChildOffset;
				}
			}
			else {
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
		}
		return { closetLength,closetPoint };
	}

	bool BVHAccel::Visible(Vertex vOberveVertex, Vertex vVertex, float margin = 0) {
		if (margin > 0) {
			Vertex temp = vVertex;
			float tempAdd = 1;
			temp.Position.z += tempAdd;
			if (vOberveVertex.Position.x > 0)
				temp.Position.x += tempAdd;
			else
				temp.Position.x += -tempAdd;
			if (vOberveVertex.Position.y > 0)
				temp.Position.y += tempAdd;
			else
				temp.Position.y += -tempAdd;
			vVertex = temp;
		}

		Ray ray(vOberveVertex.Position, vVertex.Position - vOberveVertex.Position);
		float current_t =glm::length(vVertex.Position - vOberveVertex.Position);
		SurfaceInteraction isect;
		if (!Intersect(ray, &isect))
			return false;
		glm::vec3 hitPosition = isect.pHit;
		if (current_t <=isect.t)
		{
			return true;
		}
		return false;
	}

private:
    const SplitMethod splitMethod;
    LinearBVHNode* nodes = NULL;
	std::vector<Tri*> orderedTriangles;

    BVHBuildNode* recursiveBuildTree(std::vector<Tri>& p, int start, int end,
        int* totalNodes,
		std::vector<Tri*>& orderedObjects)
    {
        // return value,the earliest value -> root
        BVHBuildNode* node = new BVHBuildNode();
        (*totalNodes)++;
        int nObjects = end - start;
  //      int nTris = end - start;
		//for (int i = start; i < end; i++) {
		//	nTris += p[i]->vertices.size();
		//}

        // judge the status of recur
        if (nObjects == 1) {
            node->initLeaf(p[start].bounds, orderedTriangles.size(), nObjects);
            for (int i = start; i < end; ++i)
				orderedTriangles.push_back(&p[i]);
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
                std::nth_element(&p[start], &p[mid], &p[end - 1] + 1,
                    [dim](Tri a, Tri b) {
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
                    int b = nBuckets * bounds.Offset(p[i].bounds.getCentroid())[dim];
                    if (b == nBuckets)
                        b -= 1;
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
                    cost[i] = 0.125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) / bounds.SurfaceArea();
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
                float leafCost = nObjects;
                if (nObjects > MAX_TRIANGLES_IN_NODE || minCost < leafCost) {
					Tri* pMid = std::partition(&p[start], &p[end - 1] + 1, [=](Tri pi) {
                        int b = nBuckets * bounds.Offset(pi.bounds.getCentroid())[dim];
                        if (b == nBuckets)
                            b -= 1;
                        return b <= minCostSplitBucket;
                    });
                    mid = pMid - &p[0];
                    if (mid == start || mid == end) { // judge if buckets devision failed
                        mid = (start + end) / 2;
                        std::nth_element(
                            &p[start], &p[mid], &p[end - 1] + 1, [dim](Tri a, Tri b) {
                                return a.bounds.getCentroid()[dim] < b.bounds.getCentroid()[dim];
                            });
                    }
                } else {
                    node->initLeaf(bounds, orderedTriangles.size(), nObjects);
                    for (int i = start; i < end; ++i)
						orderedTriangles.push_back(&p[i]);
                    return node;
                }
            }
            node->initInterior(
                dim, recursiveBuildTree(p, start, mid, totalNodes, orderedTriangles),
                recursiveBuildTree(p, mid, end, totalNodes, orderedTriangles));
        }
        return node;
    }

    int flattenBVHTree(BVHBuildNode* node, int* offset, int parentOffset)
    {
        totalLinearNodes++;
        LinearBVHNode* linearBVHNode = &nodes[*offset];
        linearBVHNode->parentOffset = parentOffset;
        linearBVHNode->bounds = node->bounds;
        int myOffset = (*offset)++;
        if (node->leftChild != NULL && node->rightChild != NULL) {
            linearBVHNode->axis = node->splitAxis;
            flattenBVHTree(node->leftChild, offset, myOffset);
            linearBVHNode->secondChildOffset = flattenBVHTree(node->rightChild, offset, myOffset);
            linearBVHNode->nObject = 0;
            linearBVHNode->objectOffset = -1;
        } else {
            linearBVHNode->objectOffset = node->index;
            linearBVHNode->nObject = node->nObjects;
        }
        float x = (linearBVHNode->bounds.pMax[0] - linearBVHNode->bounds.pMin[0]) / 2;
        float y = (linearBVHNode->bounds.pMax[1] - linearBVHNode->bounds.pMin[1]) / 2;
        float z = (linearBVHNode->bounds.pMax[2] - linearBVHNode->bounds.pMin[2]) / 2;
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

#endif // BVHTREE_HEADER