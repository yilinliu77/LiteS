
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
#include <array>
#include <fstream>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <iterator>
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

void colorizeError(const std::vector<float>& vDistances,
                   const std::vector<Vertex>& vInMeshVertices,
                   std::vector<float>& vColors, CMesh* vGTModel,
                   ACCEL::BVHAccel& vGTBVHTree) {
  const float errorThresholdMax = 0.05f;

  CMesh* outColorMesh = new CPointCloudMesh(vGTModel->vertices);
  std::vector<Vertex>& outVertices = outColorMesh->vertices;

  tbb::parallel_for(tbb::blocked_range<size_t>(size_t(0), vDistances.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i < r.end(); ++i) {
                        std::pair<float, glm::vec3> closestResult =
                            vGTBVHTree.KNearest(vInMeshVertices[i].Position);
                      }
                    });
}

void calculateAccuracy(CMesh* vPointCloud, CMesh* vGTModel,
                       ACCEL::BVHAccel& vBVHTree) {
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
  std::cout << "90% Distances Below: "
            << distances[pointIndexes[pointIndexes.size() / 10 * 1]]
            << std::endl;
  std::cout << "95% Distances Below: "
            << distances[pointIndexes[pointIndexes.size() / 100 * 5]]
            << std::endl;
  std::cout << "Average Distances is: "
            << std::accumulate(distances.begin(), distances.end(), 0.f) /
                   distances.size()
            << std::endl;
  std::cout << "Max Distances is: "
            << *std::max_element(distances.begin(), distances.end())
            << std::endl;
  std::cout << std::endl;

  return;
}

void calculateCompleteness(CMesh* vPointCloud, CMesh* vGTModel,
                           ACCEL::BVHAccel& vBVHTree) {
  size_t numPoints = vGTModel->vertices.size();
  std::vector<float> distances(numPoints);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(size_t(0), numPoints),
      [&](const tbb::blocked_range<size_t>& r) {
        for (size_t pointIndex = r.begin(); pointIndex < r.end();
             pointIndex++) {
          std::pair<float, glm::vec3> closestResult =
              vBVHTree.KNearest(vGTModel->vertices[pointIndex].Position);

          distances[pointIndex] = closestResult.first;
        }
      });
  std::vector<size_t> pointBelow50(numPoints, 0);
  std::vector<size_t> pointBelow75(numPoints, 0);
  for (size_t i = 0; i < numPoints; i++) {
    if (distances[i] < 0.050f) pointBelow50[i] = 1;
    if (distances[i] < 0.075f) pointBelow75[i] = 1;
  }

  std::cout << "Below 0.050m: "
            << std::accumulate(pointBelow50.begin(), pointBelow50.end(), 0.f) /
                   distances.size()
            << std::endl;
  std::cout << "Below 0.075m: : "
            << std::accumulate(pointBelow75.begin(), pointBelow75.end(), 0.f) /
                   distances.size()
            << std::endl;
  std::cout << "Average Distances is: "
            << std::accumulate(distances.begin(), distances.end(), 0.f) /
                   distances.size()
            << std::endl;
  std::cout << "Max Distances is: "
            << *std::max_element(distances.begin(), distances.end())
            << std::endl;

  return;
}

void upsample(CMesh* vInMesh, CMesh* vOutMesh, size_t vTargetNumPoints) {
  const std::vector<Vertex>& inVertices = vInMesh->vertices;
  assert(vInMesh->indices.size() % 3 == 0);

  std::vector<float> faceSurfaceArea(vInMesh->indices.size() / 3, 0.f);
  float totalSurfaceArea = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(size_t(0), vInMesh->indices.size() / 3), 0.f,
      [&](const tbb::blocked_range<size_t>& r, float vValue) -> float {
        float localArea = 0.f;
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const glm::vec3& pos1 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 0]].Position;
          const glm::vec3& pos2 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 1]].Position;
          const glm::vec3& pos3 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 2]].Position;

          glm::vec3 edge1 = pos2 - pos1;
          glm::vec3 edge2 = pos3 - pos1;

          float t = glm::length(glm::cross(edge1, edge2)) / 2;
          localArea += t;
          faceSurfaceArea[i] = t;
        }
        return localArea + vValue;
      },
      [](float a, float b) -> float { return a + b; });

  float targetSurfaceAreaPerFace = totalSurfaceArea / vTargetNumPoints;

  std::mt19937 gen;
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  std::vector<Vertex> outVertices;
  size_t counter = 0;
  tbb::spin_mutex mt;

  tbb::parallel_for(
      tbb::blocked_range<size_t>(size_t(0), vInMesh->indices.size() / 3),
      [&](const tbb::blocked_range<size_t>& r) {
        std::vector<Vertex> localVertices;

        for (size_t i = r.begin(); i < r.end(); i++) {
          gen.seed(i);
          const glm::vec3& pos1 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 0]].Position;
          const glm::vec3& pos2 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 1]].Position;
          const glm::vec3& pos3 =
              vInMesh->vertices[vInMesh->indices[i * 3 + 2]].Position;

          float faceSamples = faceSurfaceArea[i] / targetSurfaceAreaPerFace;
          size_t faceNumSamples = static_cast<size_t>(faceSamples);

          if (dist(gen) < (faceSamples - static_cast<float>(faceNumSamples))) {
            faceNumSamples += 1;
          }

          for (size_t j = 0; j < faceNumSamples; j++) {
            float r1 = dist(gen);
            float r2 = dist(gen);

            float tmp = std::sqrt(r1);
            float u = 1.0f - tmp;
            float v = r2 * tmp;

            float w = 1.0f - v - u;

            localVertices.push_back(Vertex(u * pos1 + v * pos2 + w * pos3,
                                           glm::vec3(0.f), glm::vec3(0.f)));
          }
        }

        mt.lock();
        counter += localVertices.size();
        outVertices.insert(outVertices.end(), localVertices.begin(),
                           localVertices.end());
        std::cout << "Up Sample " << counter << "/" << vTargetNumPoints
                  << std::endl;
        mt.unlock();
      });

  std::cout << "Sample Done" << std::endl;
  vOutMesh = new CPointCloudMesh(outVertices);
  // vOutMesh->saveMesh(vOutMesh, "../../../my_test/1.ply");
  return;
}

void CEvaluatePointCloudComponent::extraAlgorithm() {
  CMesh* pointCloud = this->m_Scene->m_Models.at("point");
  CMesh* meshModel = this->m_Scene->m_Models.at("gt_mesh");
  ACCEL::BVHAccel gtBVHTree(meshModel);
  // ACCEL::BVHAccel inBVHTree(pointCloud);

  calculateAccuracy(pointCloud, meshModel, gtBVHTree);

  // CMesh* gtUpSample;
  // upsample(pointCloud, gtUpSample,10 * pointCloud->vertices.size());
  // calculateCompleteness(pointCloud, gtUpSample, inBVHTree);

  CMesh* outMesh;
  // outMesh->saveMesh(outMesh, "../../../my_test/inside_cleaned.ply");

  std::cout << "Extra Algorithm done" << endl;

  this->waitForStepSignal();
  return;
}

void CEvaluatePointCloudComponent::extraInit() {}
