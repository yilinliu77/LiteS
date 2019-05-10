
#include "CEvaluatePointCloud.h"
#include <CEngine.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_scheduler_init.h>
#include <algorithm>
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

CEvaluatePointCloudComponent::CEvaluatePointCloudComponent(
    const map<string, CPass*>& vPass, CScene* vScene,
    const std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene, vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
}

CEvaluatePointCloudComponent::~CEvaluatePointCloudComponent() = default;

void calculateAccuracy(CMesh* vPointCloud, CModel* vGTModel,
                       BVHACCEL::BVHAccel& vBVHTree) {
  size_t numPoints = vPointCloud->vertices.size();
  std::vector<float> distances(numPoints);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(size_t(0), numPoints),
      [&](const tbb::blocked_range<size_t>& r) {
        for (size_t pointIndex = r.begin(); pointIndex < r.end();
             pointIndex++) {
          std::pair<float, glm::vec3> closestResult =
              vBVHTree.KNearest(vPointCloud->vertices[pointIndex].Position);
          distances[pointIndex] = closestResult.first;
        }
      });
  std::vector<size_t> pointIndexes(numPoints);
  std::iota(pointIndexes.begin(), pointIndexes.end(), 0);
  std::sort(pointIndexes.begin(), pointIndexes.end(),
            [&](const size_t a, const size_t b) -> bool {
              return distances[a] > distances[b];
            });
  std::cout << "Distances: "
            << distances[pointIndexes[pointIndexes.size() / 10 * 1]]
            << std::endl;
  std::cout << "Distances: "
            << std::reduce(distances.begin(), distances.end()) /
                   distances.size()
            << std::endl;

  return;
}

void CEvaluatePointCloudComponent::extraAlgorithm() {
  CMesh* pointCloud = this->m_Scene->m_Models.at("point")->meshes[0];
  CModel* meshModel = this->m_Scene->m_Models.at("gt_mesh");
  BVHACCEL::BVHAccel bvhTree(meshModel->meshes);

  calculateAccuracy(pointCloud, meshModel, bvhTree);

  CMesh* outMesh;
  // outMesh->saveMesh(outMesh, "../../../my_test/inside_cleaned.ply");

  std::cout << "Extra Algorithm done" << endl;

  this->waitForStepSignal();
  return;
}

void CEvaluatePointCloudComponent::extraInit() {}
