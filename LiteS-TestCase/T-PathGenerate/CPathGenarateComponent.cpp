#include "CPathGenarateComponent.h"
#include <CEngine.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>

#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <string>

#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>
#include "kernel.h"

#include "CBVHACCEL.h"
#include "CCDataStructure.h"
#include "CCVectorArray.cuh"
#include "CPointCloudMesh.h"
#include "trajectory_util.h"
#include "util.h"

#define THREADSPERBLOCK 128

CPathGenarateComponent::CPathGenarateComponent(const map<string, CPass*>& vPass,
                                               CScene* vScene,
                                               const std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene, vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
}

void visulizeVisibility(CMesh* vCameraMesh,
                        const thrust::host_vector<float>& vRenconScore) {
  for (size_t i = 0; i < vRenconScore.size(); i++) {
    if (vRenconScore[i] > 0)
      vCameraMesh->changeColor(glm::vec3(0.f, 1.f, 0.f), i);
  }
}

void visulizeReconstruction(CMesh* vPointCloud,
                            const thrust::host_vector<float>& vRenconScore) {
  float maxRecon =
      *thrust::max_element(vRenconScore.begin(), vRenconScore.end());
  float minRecon =
      *thrust::min_element(vRenconScore.begin(), vRenconScore.end());
  float averageScore = thrust::reduce(vRenconScore.begin(), vRenconScore.end(),
                                      (float)0, thrust::plus<float>()) /
                       vRenconScore.size();
  float dimRecon = maxRecon - minRecon;
  for (size_t i = 0; i < vRenconScore.size(); i++) {
    float colorLevel = (vRenconScore[i] - minRecon) / dimRecon;
    vPointCloud->changeColor(
        glm::vec3(colorLevel * 255.f, colorLevel * 255.f, colorLevel * 255.f),
        i);
  }
  std::cout << "Max :" << maxRecon << std::endl;
  std::cout << "Min :" << minRecon << std::endl;
  std::cout << "Average :" << averageScore << std::endl;
}

void CPathGenarateComponent::extraAlgorithm() {
  CMesh* pointCloud = this->m_Scene->m_Models.at("proxy_point");
  size_t numPoints = pointCloud->vertices.size();
  thrust::device_vector<CCDataStructure::Point> dPointCloud;
  CCDataStructure::createDevicePointCloud(pointCloud, dPointCloud);
  CCDataStructure::Point* dPointsPtr =
      thrust::raw_pointer_cast(&dPointCloud[0]);

  ACCEL::BVHAccel* bvhTree =
      new ACCEL::BVHAccel(this->m_Scene->m_Models.at("proxy_mesh"));

  CCDataStructure::DBVHAccel* dBVHPointer =
      CCDataStructure::createDBVHAccel(bvhTree);

  CCDataStructure::CCVectorArray<glm::vec4> myObsRays(numPoints, 64);

  thrust::device_vector<float> dReconScore =
      CCDataStructure::createDeviceVectorFloat(int(numPoints));
  float* dReconScorePointer = thrust::raw_pointer_cast(&dReconScore[0]);

  dim3 grid(CCDataStructure::divup(numPoints, THREADSPERBLOCK));
  dim3 block(THREADSPERBLOCK);

  // 1 Generate Nadir view
  std::vector<Vertex> cameraVertexVector;
  LiteS_Trajectory::generateNadir(glm::vec3(30, 30, 30), glm::vec3(-30, -30, 0),
                                  90.f, 0.8, cameraVertexVector);
  CMesh* cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.f, 1.f, 0.f), 10.f);
  cameraMesh->isRenderNormal = true;
  std::unique_lock<std::mutex> ul(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));
  ul.unlock();

  std::cout << std::endl << "Nadir Done" << std::endl;
  this->waitForStepSignal();

  // 2 Update rays and evaluate re constructibility
  size_t countUpdate = 0;

#pragma omp parallel
  {
    cudaStream_t stream;
    CUDACHECKERROR(cudaStreamCreate(&stream));

    cudaEvent_t event;
    CUDACHECKERROR(cudaEventCreateWithFlags(
        &event, cudaEventDefault | cudaEventDisableTiming));
#pragma omp for
    for (size_t i = 0; i < cameraVertexVector.size(); ++i) {
      updateObsRays(
          grid, block, stream, dBVHPointer, dPointsPtr, numPoints,
          myObsRays.data,
          CCDataStructure::glmToFloat3(cameraVertexVector[i].Position),
          CCDataStructure::glmToFloat3(cameraVertexVector[i].Normal));
      CUDACHECKERROR(cudaEventRecord(event, stream));
      while (cudaEventQuery(event) == cudaErrorNotReady) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      // cudaEventSynchronize(event);
      CUDACHECKERROR(cudaEventQuery(event));

#pragma omp atomic
      countUpdate += 1;
#pragma omp atomic
      std::cout << countUpdate << "/" << cameraVertexVector.size()
                << "			";
    }
  }

  CUDACHECKERROR(cudaDeviceSynchronize());
  std::cout << std::endl << "Update Done" << std::endl;
  // 3 Evaluate Reconstrucbility
  evaluateReconstrucbility(grid, block, 0, numPoints, myObsRays.data,
                           dReconScorePointer, dPointsPtr);

  CUDACHECKERROR(cudaDeviceSynchronize());
  thrust::host_vector<float> hScore = dReconScore;

  std::cout << "Evaluate Done" << std::endl;
  // 4 Visualize point
  // visulizeVisibility(pointCloud, hScore);
  visulizeReconstruction(pointCloud, hScore);

  // for (int i = 0; i < 50; ++i) std::cout << hRays[i].x << std::endl;

  // initVariebles();

  // generate_nadir();

  ////simplexPoint();

  // optimize_nadir();

  std::cout << "Extra Algorithm done" << endl;

  this->waitForStepSignal();
  return;
}

void CPathGenarateComponent::extraInit() {}
