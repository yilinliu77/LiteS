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

const float overlap = 0.8;
const float fov = 60;
const float aspect = 1.5;

CGenerateNadirComponent::CGenerateNadirComponent(
    const map<string, CPass*>& vPass, CScene* vScene, std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene,vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
  proxyPoint = vScene->m_Models.at("proxy_point")->meshes[0];
}

void writeFlightLog(std::vector<Vertex>& vVertexVector, string vLogPath,
                    string vLogPathUnreal) {
  /*
  yaw=0 -> +y
  +y -> -x == yaw increase
  y = -y
  */
  ofstream fileOut;
  ofstream fileOutUnreal;
  fileOut.open(vLogPath, ios::out);
  fileOutUnreal.open(vLogPathUnreal, ios::out);
  int imageIndex = 0;
  for (auto& item : vVertexVector) {
    float x = -item.Position[0] * 100;
    float y = item.Position[1] * 100;
    float z = item.Position[2] * 100;

    char s[30];
    sprintf_s(s, "%04d.jpg", imageIndex);
    string imageName(s);
    fileOut << imageName << "," << item.Position[0] << "," << item.Position[1]
            << "," << item.Position[2] << std::endl;

    glm::vec3 direction(-item.Normal[0], item.Normal[1], item.Normal[2]);
    direction = glm::normalize(direction);

    float yaw = 0.f;
    if (direction[0] != 0.f) yaw = std::atan(direction[1] / direction[0]);
    if (direction[0] < 0) yaw += 3.1415926;
    float pitch = 0.f;
    if (direction[0] * direction[0] + direction[1] * direction[1] > 1e-3 ||
        std::abs(direction[2]) > 1e-3)
      pitch = -std::atan(direction[2] / std::sqrt(direction[0] * direction[0] +
                                                  direction[1] * direction[1]));

    pitch = pitch / 3.1415926 * 180;
    yaw = yaw / 3.1415926 * 180;

    // postAsiaPitchYaw(pitch, yaw);

    fileOutUnreal << imageName << "," << x << "," << y << "," << z << ","
                  << pitch << "," << 0 << "," << yaw << std::endl;

    imageIndex++;
  }
}

void CGenerateNadirComponent::generate_nadir() {
  glm::vec3 pMax(66, 50, 28), pMin(-66, -50, 0);
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

  writeFlightLog(cameraVertexVector 
	  ,"../../../../my_test/nadir.log"
	  ,"../../../../my_test/nadirUnreal.log" );

  CPointCloudMesh* cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(1.0f, 0.0f, 0.0f), 10);

  CModel* cameraModel = new CModel;
  cameraModel->isRender = true;
  cameraModel->meshes.push_back(cameraMesh);

  cameraModel->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraModel));
}

void CGenerateNadirComponent::extraAlgorithm() {
  generate_nadir();

  return;
}

void CGenerateNadirComponent::extraInit() {}
