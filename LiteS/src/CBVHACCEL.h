#ifndef BVHTREE_HEADER
#define BVHTREE_HEADER

#include <Eigen/Dense>
#include <iostream>

#include "Mesh.h"

using namespace Eigen;

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

    BVHAccel(const std::vector<shared_ptr<Mesh>>& p):splitMethod(SAH){
        if (p.size() == 0)
            return;

        totalLinearNodes = 0;

        // init
        BVHBuildNode* root;
        std::vector<shared_ptr<Mesh>> pClone(p);
        int totalNodes = 0;
        root = recursiveBuildTree(pClone, 0, pClone.size(), &totalNodes, orderedMesh);

        // compact the tree
        nodes = (LinearBVHNode*)malloc(sizeof(LinearBVHNode) * totalNodes);
        int offset = 0;
        flattenBVHTree(root, &offset, -1);
    }
    ~BVHAccel() {}
    LinearBVHNode* getLinearNodes() { return nodes; }
    std::vector<shared_ptr<Mesh>> getOrderedMesh() { return orderedMesh; }

	bool BVHAccel::Intersect(Ray &ray, SurfaceInteraction *isect) const {
		if (!nodes) return false;
		bool hit = false;
		Vector3f invDir(1 / ray.d[0], 1 / ray.d[1], 1 / ray.d[2]);
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
						if (orderedMesh[node->objectOffset + i]->Intersect(
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

private:
    const SplitMethod splitMethod;
    LinearBVHNode* nodes = NULL;
	std::vector<shared_ptr<Mesh>> orderedMesh;

    BVHBuildNode* recursiveBuildTree(std::vector<shared_ptr<Mesh>>& p, int start, int end,
        int* totalNodes,
        std::vector<shared_ptr<Mesh>>& orderedObjects)
    {
        // return value,the earlist value -> root
        BVHBuildNode* node = new BVHBuildNode();
        (*totalNodes)++;
        int nObjects = end - start;
        int nTris = 0;
        for (int i = start; i < end; i++) {
            nTris += p[i]->points.size();
        }

        // judge the status of recur
        if (nObjects == 1) {
            node->initLeaf(p[start]->getBounds(), orderedMesh.size(), nObjects);
            for (int i = start; i < end; ++i)
                orderedMesh.push_back(p[i]);
            return node;
        } else {
            // compute all the bounds now between start and end
            Bounds3f bounds = p[start]->getBounds();
            for (int i = start + 1; i < end; i++)
                bounds = bounds.unionBounds(p[i]->getBounds());
            // detect if the bounds have zero volume
            // TODO
            // if (bounds.pMax==bounds.pMin)
            //{

            //}

            // select which axis should be dimension
            int dim = bounds.MaximumExtent();

            // partition
            int mid = -1;
            // when the number of object is less than 4,use euqal set strategy
            if (nObjects <= 4) {
                mid = (start + end) / 2;
                std::nth_element(&p[start], &p[mid], &p[end - 1] + 1,
                    [dim](shared_ptr<Mesh> a, shared_ptr<Mesh> b) {
                        return a->getCentroid()[dim] < b->getCentroid()[dim];
                    });
            } else {
                const int nBuckets = 12;
                struct BucketInfo {
                    Bounds3f bounds;
                    int count = 0;
                };
                BucketInfo buckets[nBuckets];
                for (int i = start; i < end; ++i) {
                    int b = nBuckets * bounds.Offset(p[i]->getBounds().getCentroid())[dim];
                    if (b == nBuckets)
                        b -= 1;
                    ++buckets[b].count;
                    buckets[b].bounds = buckets->bounds.unionBounds(p[i]->getBounds());
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
                if (nTris > MAX_TRIANGLES_IN_NODE || minCost < leafCost) {
					shared_ptr<Mesh>* pMid = std::partition(&p[start], &p[end - 1] + 1, [=](shared_ptr<Mesh> pi) {
                        int b = nBuckets * bounds.Offset(pi->getBounds().getCentroid())[dim];
                        if (b == nBuckets)
                            b -= 1;
                        return b <= minCostSplitBucket;
                    });
                    mid = pMid - &p[0];
                    if (mid == start || mid == end) { // judge if buckets devision failed
                        mid = (start + end) / 2;
                        std::nth_element(
                            &p[start], &p[mid], &p[end - 1] + 1, [dim](shared_ptr<Mesh> a, shared_ptr<Mesh> b) {
                                return a->getCentroid()[dim] < b->getCentroid()[dim];
                            });
                    }
                } else {
                    node->initLeaf(bounds, orderedMesh.size(), nObjects);
                    for (int i = start; i < end; ++i)
                        orderedMesh.push_back(p[i]);
                    return node;
                }
            }
            node->initInterior(
                dim, recursiveBuildTree(p, start, mid, totalNodes, orderedMesh),
                recursiveBuildTree(p, mid, end, totalNodes, orderedMesh));
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
        Eigen::Vector3f boundVerticesVector[] = {
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(x, y, z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(x, -y, z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(x, -y, -z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(x, y, -z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(-x, -y, -z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(-x, -y, z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(-x, y, z),
            linearBVHNode->bounds.getCentroid() + Eigen::Vector3f(-x, y, -z),
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