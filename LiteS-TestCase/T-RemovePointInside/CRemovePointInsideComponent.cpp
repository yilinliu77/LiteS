
#include "CRemovePointInsideComponent.h"
#include <CEngine.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_scheduler_init.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <string>

#include "CBVHACCEL.h"
#include "CPointCloudMesh.h"
#include "util.h"

CPathGenarateComponent::CPathGenarateComponent(const map<string, CPass*>& vPass,
                                               CScene* vScene,
                                               const std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene, vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::extraAlgorithm() {
  CMesh* pointCloud = this->m_Scene->m_Models.at("point");
  CMesh* meshModel = this->m_Scene->m_Models.at("gt_mesh");
  ACCEL::BVHAccel bvhTree(meshModel);

  const glm::vec3& boundsMax = pointCloud->bounds.pMax;
  const glm::vec3& boundsMin = pointCloud->bounds.pMin;
  const size_t numPoint = pointCloud->vertices.size();

  std::vector<Vertex> outVertex;
  std::vector<bool> pointSeen(numPoint, false);

  const float height = boundsMax[2] + 10.f;
  std::vector<glm::vec3> cameraPos;
  std::vector<Vertex> cameraVertex;
  float step = 30.f;
  for (float x = boundsMin[0] - 60.f; x < boundsMax[0] + 60.f; x += step) {
    for (float y = boundsMin[1] - 60.f; y < boundsMax[1] + 60.f; y += step) {
      cameraPos.push_back(glm::vec3(x, y, height));
      cameraVertex.push_back(Vertex(cameraPos.back(), -cameraPos.back(),
                                    glm::vec3(1.f, 0.f, 0.f)));
    }
  }

  CMesh* cameraMesh =
      new CPointCloudMesh(cameraVertex, glm::vec3(1.f, 0.f, 0.f), 5.f);
  cameraMesh->isRenderNormal = true;
  std::unique_lock<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));
  lg.unlock();

  size_t cameraIndex = 0;
  for (auto& itemCamera : cameraPos) {
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, numPoint),
        [&](const tbb::blocked_range<size_t>& r) {
          for (size_t i = r.begin(); i != r.end(); ++i) {
            if (pointSeen[i]) continue;
            const glm::vec3& pointPos = pointCloud->vertices[i].Position;
            Ray ray(itemCamera, glm::normalize(pointPos - itemCamera));
            SurfaceInteraction sr;
            if (bvhTree.Intersect(ray, &sr)) {
              if (glm::length(sr.pHit - pointPos) < 1e-1f) pointSeen[i] = true;
            } else
              pointSeen[i] = true;
          }
        });
    std::cout << cameraIndex << "/" << cameraPos.size() << std::endl;
    cameraIndex += 1;
  }

  for (size_t i = 0; i < pointCloud->vertices.size(); ++i) {
    if (pointSeen[i]) outVertex.push_back(pointCloud->vertices[i]);
  }
  CMesh* outMesh = new CPointCloudMesh(outVertex);
  outMesh->saveMesh(outMesh, "../../../my_test/inside_cleaned.ply");

  std::cout << "Reserve : "
            << std::accumulate(pointSeen.begin(), pointSeen.end(), 0) /
                   pointSeen.size()
            << std::endl;

  std::cout << "Extra Algorithm done" << endl;

  this->waitForStepSignal();
  return;
}

void CPathGenarateComponent::extraInit() {}
