#include "CGenerateNadirComponent.h"
#include <CEngine.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <glm/gtc/matrix_access.hpp>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include "CBVHACCEL.h"
#include "CPointCloudMesh.h"
#include "util.h"
#include "trajectory_util.h"

const float overlap = 0.8;
const float fov = 60;
const float aspect = 1.5;

CGenerateNadirComponent::CGenerateNadirComponent(
    const map<string, CPass*>& vPass, CScene* vScene, std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene,vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
  proxyPoint = vScene->m_Models.at("proxy_point");
}

void CGenerateNadirComponent::generate_nadir() {
  glm::vec3 pMax(70, 70, 40), pMin(-70, -70, 0);
  //pMax = glm::vec3(64, 46, 30);
  //pMin = glm::vec3(-65, -45, 0);

  glm::vec3 mesh_dim = pMax - pMin;
  float max_height = pMax[2] / 2 * 3;

  std::vector<Vertex> cameraVertexVector;

  float stepx =
      max_height / std::tan(glm::radians(fov) / 2) * 2 * (1 - overlap);
  float stepy = stepx / aspect;

  glm::vec3 startPos =
      pMin - glm::vec3(mesh_dim[0] / 2, mesh_dim[1] / 2, 0);
  glm::vec3 endPos =
      pMax + glm::vec3(mesh_dim[0] / 2, mesh_dim[1] / 2, 0);

  glm::vec3 cameraPos = startPos;
  cameraPos.z = max_height;
  while (cameraPos.x < endPos.x) {
    while (cameraPos.y < endPos.y) {
      Vertex v;
      v.Position = cameraPos;
      v.Normal = glm::normalize(-cameraPos);
      cameraVertexVector.push_back(v);
      cameraPos.y += stepy;
    }
    cameraPos.x += stepx;
    cameraPos.y = startPos.y;
  }

  LiteS_Trajectory::saveTrajectory("../../../../my_test/nadir.log",
                                   cameraVertexVector);
  LiteS_Trajectory::saveTrajectoryUnreal("../../../../my_test/nadirUnreal.log",
                                   cameraVertexVector,true);

  CPointCloudMesh* cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(1.0f, 0.0f, 0.0f), 10);

  cameraMesh->isRender = true;

  cameraMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));
}

void CGenerateNadirComponent::extraAlgorithm() {
  generate_nadir();

  return;
}

void CGenerateNadirComponent::extraInit() {}
