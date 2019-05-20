#include "CBVHDisplay.h"
#include <CEngine.h>
#include "CBVHACCEL.h"
#include "CPointCloudMesh.h"

#include <queue>
#include <array>
#include <random>
#include <set>
#include "util.h"

CMesh* proxyPoint;
ACCEL::BVHAccel* bvhTree;
CMesh* cameraPosMesh;
vector<Vertex> cameraPosVector;

// Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y,
// Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;
vector<vector<int>> seenSample;

CBVHDisplay::CBVHDisplay(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene, vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
}

CBVHDisplay::~CBVHDisplay() = default;

void CBVHDisplay::extraAlgorithm() {
  CMesh* proxyMesh = this->m_Scene->m_Models.at("proxy_mesh");
  CMesh* proxyPoint = this->m_Scene->m_Models.at("proxy_point");
  ACCEL::BVHAccel bvh(proxyMesh);

  const std::vector<ACCEL::LinearBVHNode>& nodes = bvh.getLinearNodes();
  const std::vector<Tri>& tris = bvh.getOrderedTriangles();

  std::queue<int> nodeToVisit;
  int currentNodeIndex = 0;
  nodeToVisit.push(currentNodeIndex);
  int boundIndex = 0;
  while (true) {
    const ACCEL::LinearBVHNode* node = &nodes[currentNodeIndex];
    // Check ray against BVH node
    if (node->nObject > 0) {
      // Intersect ray with primitives in leaf BVH node
      for (int i = 0; i < node->nObject; ++i) {
      }
      if (nodeToVisit.size() == 0) break;
      currentNodeIndex = nodeToVisit.front();
      nodeToVisit.pop();
    } else {
      vector<Vertex> t;

      for (size_t i = 0; i < 36; ++i) {
        glm::vec3 position(
            nodes[currentNodeIndex + 1].boundVertices[i * 3],
            nodes[currentNodeIndex + 1].boundVertices[i * 3 + 1],
            nodes[currentNodeIndex + 1].boundVertices[i * 3 + 2]);
        t.push_back(Vertex(position, glm::vec3(0.f), glm::vec3(0.f)));
      }

      for (size_t i = 0; i < 36; ++i) {
        glm::vec3 position(
            nodes[node->secondChildOffset].boundVertices[i * 3],
            nodes[node->secondChildOffset].boundVertices[i * 3 + 1],
            nodes[node->secondChildOffset].boundVertices[i * 3 + 2]);
        t.push_back(Vertex(position, glm::vec3(0.f), glm::vec3(0.f)));
      }

      CMesh* newMesh = new CTriMesh(t);
      newMesh->isLineRender = true;
      std::unique_lock<std::mutex> lg(CEngine::m_addMeshMutex);
      CEngine::toAddModels.push_back(std::make_pair(
          std::string("bound") + LiteUtil::numberToString(boundIndex++),
          newMesh));
      lg.unlock();
      this->waitForStepSignal();

      nodeToVisit.push(currentNodeIndex + 1);
      nodeToVisit.push(node->secondChildOffset);

      currentNodeIndex = nodeToVisit.front();
      nodeToVisit.pop();

      this->m_Scene->m_Models.erase(std::string("bound") +
                                    LiteUtil::numberToString(boundIndex-1));

    }
  }
  std::cout << "lalala" << std::endl;
  return;
}

void CBVHDisplay::extraInit() {}
