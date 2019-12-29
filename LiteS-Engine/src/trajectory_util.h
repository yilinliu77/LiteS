#pragma once
#ifndef LITES_TRAJECTORY_UTIL_H
#define LITES_TRAJECTORY_UTIL_H

#define GLM_FORCE_SILENT_WARNINGS

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <glm/gtx/quaternion.hpp>

#include "CMesh.h"
#include "util.h"

namespace LiteS_Trajectory {
void loadTrajectoryMVESpline(const std::string vPath,
                             std::vector<Vertex>& vCameraVertexVector) {
  try {
    std::ifstream ifs(vPath, std::ios::in);
    std::string line;
    getline(ifs, line);
    getline(ifs, line);
    while (line.length() > 5) {
      float x, y, z, qx, qy, qz, qw;
      std::vector<float> splitItems = LiteUtil::splitString<float>(line, ",");
      x = splitItems[0];
      y = splitItems[1];
      z = splitItems[2];
      qw = splitItems[3];
      qx = splitItems[4];
      qy = splitItems[5];
      qz = splitItems[6];
      float key = splitItems[7];
      if (key < 0.0001f) {
        getline(ifs, line);
        continue;
      }
      glm::quat q = glm::quat(qw, qx, qy, qz);
      glm::vec3 unitVec(0.f, 0.f, 1.f);
      unitVec = unitVec * q;
      if (unitVec[2] > 0) unitVec[2] = -unitVec[2];
      vCameraVertexVector.push_back(Vertex());
      vCameraVertexVector.back().Position = glm::vec3(x, y, z);
      vCameraVertexVector.back().Normal =
          glm::normalize(glm::vec3(unitVec[0], unitVec[1], unitVec[2]));
      line = "";
      getline(ifs, line);
    }

    ifs.close();
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    throw e;
  }
}


glm::quat rot2quat(std::vector<float> const& R) {
  float v0 = 1.0 + R[0] + R[4] + R[8];
  float v1 = 1.0 + R[0] - R[4] - R[8];
  float v2 = 1.0 - R[0] + R[4] - R[8];
  float v3 = 1.0 - R[0] - R[4] + R[8];

  // TODO handle nan

  glm::quat q;

  if (v0 >= v1 && v0 >= v2 && v0 >= v3) {
    float tmp = float(0.5) * std::sqrt(v0);
    q[0] = tmp;
    q[1] = (R[7] - R[5]) / (float(4.0) * tmp);
    q[2] = (R[2] - R[6]) / (float(4.0) * tmp);
    q[3] = (R[3] - R[1]) / (float(4.0) * tmp);

    return q;
  }

  if (v1 >= v0 && v1 >= v2 && v1 >= v3) {
    float tmp = float(0.5) * std::sqrt(v1);
    q[0] = (R[7] - R[5]) / (float(4.0) * tmp);
    q[1] = tmp;
    q[2] = (R[1] + R[3]) / (float(4.0) * tmp);
    q[3] = (R[2] + R[6]) / (float(4.0) * tmp);

    return q;
  }

  if (v2 >= v0 && v2 >= v1 && v2 >= v3) {
    float tmp = float(0.5) * std::sqrt(v2);
    q[0] = (R[2] - R[6]) / (float(4.0) * tmp);
    q[1] = (R[1] + R[3]) / (float(4.0) * tmp);
    q[2] = tmp;
    q[3] = (R[5] + R[7]) / (float(4.0) * tmp);

    return q;
  }

  // if (v3 >= v0 && v3 >= v1 && v3 >= v2) {
  float tmp = float(0.5) * std::sqrt(v3);
  q[0] = (R[3] - R[1]) / (float(4.0) * tmp);
  q[1] = (R[2] + R[6]) / (float(4.0) * tmp);
  q[2] = (R[5] + R[7]) / (float(4.0) * tmp);
  q[3] = tmp;

  //return q;
}

void loadTrajectoryMVEUTJ(const std::string vPath,
                             std::vector<Vertex>& vCameraVertexVector) {
  try {
    std::ifstream ifs(vPath, std::ios::in);
    std::string line;
    float x, y, z, qx, qy, qz, qw;
	getline(ifs, line);
    vCameraVertexVector.resize(LiteUtil::StringToNumber<int>(line));

    getline(ifs, line);
    size_t index = 0;
    while (line.length() > 2) {
      std::vector<float> splitItems = LiteUtil::splitString<float>(line, " ");
      x = splitItems[0];
      y = splitItems[1];
      z = splitItems[2];

      getline(ifs, line);
      splitItems = LiteUtil::splitString<float>(line, " ");
      std::vector<float> rotateVector;
      rotateVector.push_back(splitItems[0]);
      rotateVector.push_back(splitItems[1]);
      rotateVector.push_back(splitItems[2]);
      getline(ifs, line);
      splitItems = LiteUtil::splitString<float>(line, " ");
      rotateVector.push_back(splitItems[0]);
      rotateVector.push_back(splitItems[1]);
      rotateVector.push_back(splitItems[2]);
      getline(ifs, line);
      splitItems = LiteUtil::splitString<float>(line, " ");
      rotateVector.push_back(splitItems[0]);
      rotateVector.push_back(splitItems[1]);
      rotateVector.push_back(splitItems[2]);
      getline(ifs, line);


      glm::quat q = rot2quat(rotateVector);
      glm::vec3 unitVec(0.f, 0.f, 1.f);
      unitVec = unitVec * q;
      if (unitVec[2] > 0) unitVec[2] = -unitVec[2];
      vCameraVertexVector[index].Position = glm::vec3(x, y, z);
      vCameraVertexVector[index].Normal =
          glm::normalize(glm::vec3(unitVec[0], unitVec[1], unitVec[2]));

      line = "";
      getline(ifs, line);
    }

    ifs.close();
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    throw e;
  }
}

void loadTrajectoryUnreal(const std::string vPath,
                          std::vector<Vertex>& vCameraVertexVector) {
  try {
    std::ifstream ifs(vPath, std::ios::in);
    std::string line;
    getline(ifs, line);
    while (line.length() > 5) {
      float x, y, z, dx, dy, dz;
      float pitch, yaw;
      std::vector<float> splitItems = LiteUtil::splitString<float>(line, ",");
      x = -splitItems[1] / 100.f;
      y = splitItems[2] / 100.f;
      z = splitItems[3] / 100.f;
      pitch = splitItems[4];
      yaw = splitItems[6] + 90.f;

      dz = -std::sin(pitch / 180.f * 3.14159f);
      dx = -std::cos(pitch / 180.f * 3.14159f) *
           std::cos(yaw / 180.f * 3.14159f);
      dy =
          std::cos(pitch / 180.f * 3.14159f) * std::sin(yaw / 180.f * 3.14159f);

      vCameraVertexVector.push_back(Vertex());
      vCameraVertexVector.back().Position = glm::vec3(x, y, z);
      vCameraVertexVector.back().Normal = glm::normalize(glm::vec3(dx, dy, dz));
      line = "";
      getline(ifs, line);
    }

    ifs.close();
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    throw e;
  }
}

void saveTrajectory(const std::string vPath,
                    const std::vector<Vertex>& vCameraVertexVector) {
  try {
    std::ofstream fileOut;
    fileOut.open(vPath, std::ios::out);
    int imageIndex = 0;
    for (auto& item : vCameraVertexVector) {
      float x = item.Position[0];
      float y = item.Position[1];
      float z = item.Position[2];
      //if (x > 1000 || x < -1000 || y > 1000 || y < -1000) continue;
      char s[30];
      snprintf(s,sizeof(s), "%04d.jpg", imageIndex);
      std::string imageName(s);
      fileOut << imageName << "," << item.Position[0] << "," << item.Position[1]
              << "," << item.Position[2] << std::endl;

      imageIndex++;
    }
    fileOut.close();
  } catch (std::exception* e) {
    std::cout << e->what() << std::endl;
    throw e;
  }
}

void postAsia(float& vYaw) { vYaw -= 90.f; }

void saveTrajectoryUnreal(const std::string vPath,
                          const std::vector<Vertex>& vCameraVertexVector,
                          bool isPostProcess = false) {
  try {
    std::ofstream fileOutUnreal;
    fileOutUnreal.open(vPath, std::ios::out);
    int imageIndex = 0;
    for (auto& item : vCameraVertexVector) {
      float x = -item.Position[0] * 100;
      float y = item.Position[1] * 100;
      float z = item.Position[2] * 100;
      //if (x > 100000 || x < -100000 || y > 100000 || y < -100000) continue;
      char s[30];
      snprintf(s,sizeof(s), "%04d.jpg", imageIndex);
      std::string imageName(s);

      glm::vec3 direction(-item.Normal[0], item.Normal[1], item.Normal[2]);
      direction = glm::normalize(direction);

      float yaw = 0.f;
      if (direction[0] != 0.f) yaw = std::atan(direction[1] / direction[0]);
      if (direction[0] < 0) yaw += 3.1415926f;
      float pitch = 0.f;
      if (direction[0] * direction[0] + direction[1] * direction[1] > 1e-3 ||
          std::abs(direction[2]) > 1e-3)
        pitch = std::acos(std::min(
            1.f,
            glm::dot(glm::normalize(glm::vec3(direction[0], direction[1], 0)),
                     direction)));

      pitch = pitch / 3.1415926f * 180;
      yaw = yaw / 3.1415926f * 180;

      if (isPostProcess) postAsia(yaw);

      fileOutUnreal << imageName << "," << x << "," << y << "," << z << ","
                    << pitch << "," << 0 << "," << yaw << std::endl;

      imageIndex++;
    }
    fileOutUnreal.close();
  } catch (std::exception* e) {
    std::cout << e->what() << std::endl;
    throw e;
  }
}

void generateNadir(const glm::vec3 vMax, const glm::vec3 vMin, const float vFovInDegrees,
                   const float vOverlap,
                   std::vector<Vertex>& vCameraVertexVector) {

  glm::vec3 mesh_dim = vMax - vMin;
  float max_height = vMax[2] / 2 * 3;

  float stepx = max_height / std::tan(glm::radians(vFovInDegrees) / 2) * 2 *
                (1 - vOverlap);
  float stepy = stepx / 1.0f;

  glm::vec3 startPos = vMin - glm::vec3(mesh_dim[0] / 2, mesh_dim[1] / 2, 0);
  glm::vec3 endPos = vMax + glm::vec3(mesh_dim[0] / 2, mesh_dim[1] / 2, 0);

  glm::vec3 cameraPos = startPos;
  cameraPos.z = max_height;
  while (cameraPos.x < endPos.x) {
    while (cameraPos.y < endPos.y) {
      Vertex v;
      v.Position = cameraPos;
      v.Normal = glm::normalize(-cameraPos);
      vCameraVertexVector.push_back(v);
      cameraPos.y += stepy;
    }
    cameraPos.x += stepx;
    cameraPos.y = startPos.y;
  }

  return;
}

}  // namespace LiteS_Trajectory

#endif