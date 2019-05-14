#pragma once
#ifndef LITES_TRAJECTORY_UTIL_H
#define LITES_TRAJECTORY_UTIL_H

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
	  if (x > 1000 || x < -1000 || y > 1000 || y < -1000)
		  continue;
      char s[30];
      sprintf_s(s, "%04d.jpg", imageIndex);
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

void postAsia(float& vYaw) {
	vYaw -= 90.f;
}

void saveTrajectoryUnreal(const std::string vPath,
                          const std::vector<Vertex>& vCameraVertexVector,bool isPostProcess=false) {
  try {
    std::ofstream fileOutUnreal;
    fileOutUnreal.open(vPath, std::ios::out);
    int imageIndex = 0;
    for (auto& item : vCameraVertexVector) {
      float x = -item.Position[0] * 100;
      float y = item.Position[1] * 100;
      float z = item.Position[2] * 100;
	  if (x > 100000 || x < -100000|| y > 100000 || y < -100000)
		  continue;
      char s[30];
      sprintf_s(s, "%04d.jpg", imageIndex);
      std::string imageName(s);

      glm::vec3 direction(-item.Normal[0], item.Normal[1], item.Normal[2]);
      direction = glm::normalize(direction);

      float yaw = 0.f;
      if (direction[0] != 0.f) yaw = std::atan(direction[1] / direction[0]);
      if (direction[0] < 0) yaw += 3.1415926f;
      float pitch = 0.f;
      if (direction[0] * direction[0] + direction[1] * direction[1] > 1e-3 ||
          std::abs(direction[2]) > 1e-3)
        pitch = std::acos(std::min(1.f,
            glm::dot(glm::normalize(glm::vec3(direction[0], direction[1], 0)),
                     direction)));

      pitch = pitch / 3.1415926f * 180;
      yaw = yaw / 3.1415926f * 180;

	  if (isPostProcess)
		  postAsia(yaw);

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

}  // namespace LiteS_Trajectory

#endif