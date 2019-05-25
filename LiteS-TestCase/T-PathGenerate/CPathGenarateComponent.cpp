#include "CPathGenarateComponent.h"
#include <CEngine.h>
#include <tbb/atomic.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_scheduler_init.h>
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

#define THREADSPERBLOCK 64
const size_t MAXCAMERAFORONEPOINT = 32;

const float DMAX = 60;
const float DMIN = 5;
const float DSUGGEST = 45;
const float SIMPLEXINITIALIZESCALE = 5.0f;
const int heightMapWidth = 32;
const int heightMapHeight = 32;

const string fileLogPath = "../../../my_test/camera.log";
const int totalCameraNums = 40;
float targetReconstrucbility = 15.f;

float max_building_height;
float averageSeenSample = 0;
float averageScore = 0;

// Input Mesh
CMesh* proxyPoint;
ACCEL::BVHAccel* bvhTree;

CMesh* cameraMesh;
vector<Vertex> cameraVertexVector;
CMesh* cameraCandidateMesh;

My8BitRGBImage airspace;
std::vector<std::vector<std::pair<float, float>>> airspaceHeat;
tbb::spin_mutex initRaysMutex;
std::ofstream fileFp;

// For Reconstruction
tbb::concurrent_vector<float> reconstructionScore;
std::vector<std::vector<glm::vec3>> simplexes;
vector<size_t> cameraInitializeTimes;
CMesh* cameraAdjustMesh;
std::vector<int> ranks;
// Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y,
// Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;

CPathGenarateComponent::CPathGenarateComponent(const map<string, CPass*>& vPass,
                                               CScene* vScene,
                                               const std::string vResourceDir)
    : CPointCloudComponent(vPass, vScene, vResourceDir) {
  DisplayPass = this->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::initVariebles() {
  proxyPoint = this->m_Scene->m_Models.at("proxy_point");
  obsRays.resize(proxyPoint->vertices.size());

  bvhTree = new ACCEL::BVHAccel(this->m_Scene->m_Models.at("proxy_mesh"));
  airspace.ncols = heightMapWidth;
  airspace.nrows = heightMapHeight;
  airspace.data = new float[heightMapWidth * heightMapHeight];
  for (size_t y = 0; y < heightMapHeight; ++y) {
    airspaceHeat.push_back(std::vector<std::pair<float, float>>());
    for (size_t x = 0; x < heightMapWidth; ++x) {
      airspace.data[y * heightMapWidth + x] = -9999999;
      airspaceHeat[y].push_back(std::make_pair<float, float>(0.f, 0.f));
    }
  }
  float xDim = proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0];
  float yDim = proxyPoint->bounds.pMax[1] - proxyPoint->bounds.pMin[1];

  for (size_t i = 0; i < proxyPoint->vertices.size(); ++i) {
    int x = floor(
        (proxyPoint->vertices[i].Position.x - proxyPoint->bounds.pMin[0]) /
        xDim * heightMapWidth);
    int y = floor(
        (proxyPoint->vertices[i].Position.y - proxyPoint->bounds.pMin[1]) /
        yDim * heightMapHeight);
    x = (x >= heightMapWidth ? heightMapWidth - 1 : x);
    y = (y >= heightMapHeight ? heightMapWidth - 1 : y);

    if (airspace.data[y * heightMapWidth + x] <
        proxyPoint->vertices[i].Position.z)
      airspace.data[y * heightMapWidth + x] =
          proxyPoint->vertices[i].Position.z;
    airspaceHeat[y][x].first += 1;
  }
  bool hole = true;
  while (hole) {
    hole = false;
    for (size_t y = 0; y < heightMapHeight; y++) {
      for (size_t x = 0; x < heightMapWidth; x++) {
        if (airspace.data[y * heightMapWidth + x] == -9999999) {
          hole = true;
          if (x > 1) {
            if (airspace.data[y * heightMapWidth + x - 1] != -9999999)
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[y * heightMapWidth + x - 1];
            else if (y > 1)
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[(y - 1) * heightMapWidth + x];
            else
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[(y + 1) * heightMapWidth + x];

          } else {
            if (airspace.data[y * heightMapWidth + x + 1] != -9999999)
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[y * heightMapWidth + x + 1];
            else if (y > 1)
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[(y - 1) * heightMapWidth + x];
            else
              airspace.data[y * heightMapWidth + x] =
                  airspace.data[(y + 1) * heightMapWidth + x];
          }
        }
      }
    }
  }

  max_building_height = 0;
  for (size_t y = 0; y < heightMapHeight; y++) {
    for (size_t x = 0; x < heightMapWidth; x++) {
      if (airspace.data[y * heightMapWidth + x] > max_building_height)
        max_building_height = airspace.data[y * heightMapWidth + x];
    }
  }
}

void CPathGenarateComponent::visualizeCamera(size_t vCameraIndex) {
  std::cout << "start visualize" << endl;
  cameraAdjustMesh->changeColor(glm::vec3(0, 0.5, 0.5), vCameraIndex);
  for (int i = 0; i < obsRays.size(); ++i) {
    for (int j = 0; j < obsRays[i].size(); ++j) {
      if (obsRays[i][j].first.w == vCameraIndex) {
        proxyPoint->changeColor(glm::vec3(0.0f, 0.0f, 1.0f), i);
        std::cout << "change " << i << " point color to blue" << endl;
      }
    }
  }
  std::cout << "done visualize" << endl;
  this->waitForStepSignal();
  cameraAdjustMesh->changeColor(glm::vec3(0, 1, 0), vCameraIndex);
}

float isValidPosition(glm::vec3 vCameraPos) {
  // Return 0 if to close to mesh or ground, -1 if vailed, >0 if the building is
  // higher than the position and return the height
  if (vCameraPos.z < DMIN || bvhTree->KNearest(vCameraPos).first < DMIN)
    return 0;

  // In airspace
  int y = (vCameraPos[1] - proxyPoint->bounds.pMin[1]) /
          (proxyPoint->bounds.pMax[1] - proxyPoint->bounds.pMin[1]) *
          heightMapHeight;
  int x = (vCameraPos[0] - proxyPoint->bounds.pMin[0]) /
          (proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0]) *
          heightMapWidth;
  x = (x >= heightMapWidth ? heightMapWidth - 1 : x);
  y = (y >= heightMapHeight ? heightMapWidth - 1 : y);
  if (x < 0 || y < 0 || x > proxyPoint->bounds.pMax[0] ||
      y > proxyPoint->bounds.pMax[1]) {
    return -1;
  }
  if (vCameraPos[2] < airspace.data[heightMapWidth * y + x]) {
    return airspace.data[heightMapWidth * y + x];
  }
  return -1;
}

void CPathGenarateComponent::generate_nadir() {
  glm::vec3 mesh_dim = proxyPoint->bounds.pMax - proxyPoint->bounds.pMin;
  float max_height = proxyPoint->bounds.pMax[2] + 10;

  float stepx = mesh_dim[0] / std::sqrt(totalCameraNums);
  float stepy = mesh_dim[1] / std::sqrt(totalCameraNums);

  // srand(time(0));
  // for (size_t iter = 0; iter < 100; iter++){
  //	glm::vec3 cameraPos = glm::linearRand(proxyPoint->bounds.pMin
  //		, proxyPoint->bounds.pMax+glm::vec3(0,0,20));
  //	while (-1!=isValidPosition(cameraPos))
  //	{
  //		cameraPos = glm::linearRand(proxyPoint->bounds.pMin,
  // proxyPoint->bounds.pMax);
  //	}
  //	Vertex v;
  //	v.Position = cameraPos;
  //	v.Normal = -cameraPos;
  //	cameraVertexVector.push_back(v);
  //}

  glm::vec3 startPos = proxyPoint->bounds.pMin - glm::vec3(40, 40, 0);
  glm::vec3 endPos = proxyPoint->bounds.pMax + glm::vec3(40, 40, 0);

  glm::vec3 cameraPos = startPos;
  cameraPos.z = max_height;
  while (cameraPos.x < endPos.x) {
    while (cameraPos.y < endPos.y) {
      Vertex v;
      v.Position = cameraPos;
      v.Normal = glm::normalize(glm::vec3(0, 0, -1));
      cameraVertexVector.push_back(v);
      cameraPos.y += stepy;
    }
    cameraPos.x += stepx;
    cameraPos.y = startPos.y;
  }

  cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(1.0f, 0.0f, 0.0f), 10);
  cameraAdjustMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.0f, 1.0f, 0.0f), 20);
  vector<Vertex> cameraCandidate;
  cameraCandidate.resize(cameraVertexVector.size() * 4);
  cameraCandidateMesh =
      new CPointCloudMesh(cameraCandidate, glm::vec3(0.0f, 0.0f, 1.0f), 10);
  cameraAdjustMesh->isRender = true;
  cameraCandidateMesh->isRender = true;
  cameraAdjustMesh->isRenderNormal = true;
  cameraCandidateMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraAdjustMesh));
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraCandidateMesh));
}

std::pair<float, float> heuristicEvaluete(size_t vPointIndex,
                                          glm::vec3 const& vCameraPos,
                                          glm::vec3 vCameraDirection,
                                          size_t vCameraIndex) {
  float totalScore = 0.f;
  float minPointNotSeenScore = 0.f;

  // Break if point invisible
  glm::vec3 samplePosition = proxyPoint->vertices[vPointIndex].Position;
  bool visible = false;

  visible = bvhTree->strongVisible(vCameraPos, vCameraDirection, samplePosition,
                                   DMAX);
  if (!visible) return {0.f, 0.f};

  const float currentScore = reconstructionScore[vPointIndex];
  if (currentScore > targetReconstrucbility) return {0.f, 0.f};
  if (currentScore < targetReconstrucbility)
    minPointNotSeenScore += max(targetReconstrucbility - currentScore, 0.f);

  glm::vec3 sampleNormal =
      glm::normalize(proxyPoint->vertices[vPointIndex].Normal);
  glm::vec3 sample2TargetCamera = vCameraPos - samplePosition;

  float score = 0;
  int foundReference = 0;

  for (auto cameraItem : obsRays[vPointIndex]) {
    if (cameraItem.first.w == vCameraIndex) {
      continue;
    }
    if (cameraItem.first.w < 0) continue;
    glm::vec3 camera1Pos = cameraVertexVector[cameraItem.first.w].Position;
    glm::vec3 camera1Direction = cameraVertexVector[cameraItem.first.w].Normal;
    glm::vec3 sample2Camera1 = camera1Pos - samplePosition;

    float camera1Distance = glm::length(sample2Camera1);
    float camera2Distance = glm::length(sample2TargetCamera);
    float maxDistance = max(camera1Distance, camera2Distance);

    if (maxDistance > DMAX) continue;

    float viewAngleCos = min(glm::dot(glm::normalize(sample2Camera1),
                                      glm::normalize(sample2TargetCamera)),
                             0.9999f);
    float viewAngle = std::acos(viewAngleCos);

    // w1 = 1 / ( 1 + exp( -k1 * ( alpha - alpha1 ) ) )
    float tempw1 = (viewAngle - glm::pi<float>() / 16.0f);
    float w1 = 1 / (1 + (std::exp(-32 * tempw1)));

    // w2 = min( max(d1, d2) / dmax, 1 )
    float w2 = 1 - abs(min((maxDistance - DSUGGEST), DSUGGEST)) / DSUGGEST;

    // w3 =1 - 1 / ( 1 + exp( -k3 * ( alpha - alpha3 ) ) )
    float w3 =
        1 - 1 / (1 + std::exp(-8 * (viewAngle - glm::pi<float>() / 4.0f)));

    if (glm::distance(sample2Camera1, sample2TargetCamera) < 0.01) continue;
    glm::vec3 viewPlaneNormal =
        glm::normalize(glm::cross(sample2Camera1, sample2TargetCamera));
    float theta1 =
        max(glm::dot(glm::normalize(sample2Camera1), sampleNormal), 0.f);
    float theta2 =
        max(glm::dot(glm::normalize(sample2TargetCamera), sampleNormal), 0.f);
    float theta = (theta1 + theta2) / 2;

    score += w1 * w2 * w3 * theta;
    assert(score >= 0);
  }

  // encourage seen the less seen point, don't worry divide by 0
  float punish = 1 / std::max(float(obsRays[vPointIndex].size()), 1.0f);

  // 1 prevent divide by 0
  totalScore += 1 / max(currentScore, 1.0f) * score;
  minPointNotSeenScore = 1 / max(currentScore, 1.0f) * minPointNotSeenScore;
  // totalScore += score;

  assert(totalScore >= 0);
  assert(minPointNotSeenScore >= 0);
  return {totalScore, 0};
}

// Camera Pos, Camera Direction, Camera Index
float func(const glm::vec3 const& vCameraPos, const glm::vec3 vCameraDirection,
           const size_t vCameraIndex) {
  if (isValidPosition(vCameraPos) >= 0) return -1;

  float totalScore = 0;
  float minPointNotSeenScore = 0;

  std::pair<float, float> score = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(size_t(0), proxyPoint->vertices.size()),
      std::make_pair<float, float>(0.f, 0.f),
      [=](const tbb::blocked_range<size_t>& r,
          std::pair<float, float> value) -> std::pair<float, float> {
        for (int i = r.begin(); i != r.end(); ++i) {
          std::pair<float, float> localScore =
              heuristicEvaluete(i, vCameraPos, vCameraDirection, vCameraIndex);
          value.first += localScore.first;
          value.second += localScore.second;
        }
        return value;
      },
      [](std::pair<float, float> x,
         std::pair<float, float> y) -> std::pair<float, float> {
        return std::make_pair<float, float>(x.first + y.first,
                                            x.second + y.second);
      });
  minPointNotSeenScore = score.second;
  totalScore = score.first;
  // minPointNotSeenScore = 1 / max(minPointNotSeenScore, 1.0f);
  return minPointNotSeenScore + totalScore;
};

void initRays(int vi) {
  glm::vec3 samplePos = proxyPoint->vertices[vi].Position;
  glm::vec3 sampleNormal = proxyPoint->vertices[vi].Normal;

  // Used for parallel
  vector<std::pair<glm::vec4, glm::vec4>> localObsRays;
  vector<int> localSeenSample;

  for (int j = 0; j < cameraVertexVector.size(); ++j) {
    glm::vec3 direction = cameraVertexVector[j].Position - samplePos;

    // Not occluded
    if (bvhTree->strongVisible(cameraVertexVector[j].Position,
                               -cameraVertexVector[j].Position, samplePos,
                               DMAX)) {
      localObsRays.push_back(
          std::make_pair(glm::vec4(direction, j),
                         glm::vec4(glm::normalize(glm::vec3(0, 0, -1)), 1)));
    }
  }

  obsRays[vi].insert(obsRays[vi].end(), localObsRays.begin(),
                     localObsRays.end());
}

void evalueteReconstrucbility(size_t vPointIndex) {
  reconstructionScore[vPointIndex] = 0;
  for (auto& cameraItem1 : obsRays[vPointIndex]) {
    for (auto& cameraItem2 : obsRays[vPointIndex]) {
      if (cameraItem1.first.w == cameraItem2.first.w) continue;
      glm::vec3 samplePosition = proxyPoint->vertices[vPointIndex].Position;

      glm::vec3 sampleNormal =
          glm::normalize(proxyPoint->vertices[vPointIndex].Normal);

      glm::vec3 sample2Camera1 =
          cameraVertexVector[cameraItem1.first.w].Position - samplePosition;
      glm::vec3 sample2Camera2 =
          cameraVertexVector[cameraItem2.first.w].Position - samplePosition;

      float camera1Distance = glm::length(sample2Camera1);
      float camera2Distance = glm::length(sample2Camera2);
      float maxDistance = max(camera1Distance, camera2Distance);

      if (maxDistance > DMAX) continue;

      float viewAngleCos = min(glm::dot(glm::normalize(sample2Camera1),
                                        glm::normalize(sample2Camera2)),
                               0.9999f);
      float viewAngle = std::acos(viewAngleCos);

      // w1 = 1 / ( 1 + exp( -k1 * ( alpha - alpha1 ) ) )
      float tempw1 = (viewAngle - glm::pi<float>() / 16.0f);
      float w1 = 1 / (1 + (std::exp(-32 * tempw1)));

      // w2 = min( max(d1, d2) / dmax, 1 )
      float w2 = 1 - min(maxDistance / DMAX, 1.0f);

      // w3 =1 - 1 / ( 1 + exp( -k3 * ( alpha - alpha3 ) ) )
      float w3 =
          1 - 1 / (1 + std::exp(-8 * (viewAngle - glm::pi<float>() / 4.0f)));

      if (glm::distance(sample2Camera1, sample2Camera2) < 0.01) continue;
      glm::vec3 viewPlaneNormal =
          glm::normalize(glm::cross(sample2Camera1, sample2Camera2));
      float theta1 =
          max(glm::dot(glm::normalize(sample2Camera1), sampleNormal), 0.f);
      float theta2 =
          max(glm::dot(glm::normalize(sample2Camera2), sampleNormal), 0.f);
      float theta = (theta1 + theta2) / 2;

      reconstructionScore[vPointIndex] += w1 * w2 * w3 * theta;
    }
  }

  int x = floor((proxyPoint->vertices[vPointIndex].Position.x -
                 proxyPoint->bounds.pMin[0]) /
                (proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0]) *
                heightMapWidth);
  int y = floor((proxyPoint->vertices[vPointIndex].Position.y -
                 proxyPoint->bounds.pMin[1]) /
                (proxyPoint->bounds.pMax[1] - proxyPoint->bounds.pMin[1]) *
                heightMapHeight);
  x = (x >= heightMapWidth ? heightMapWidth - 1 : x);
  y = (y >= heightMapHeight ? heightMapWidth - 1 : y);

  airspaceHeat[y][x].second += reconstructionScore[vPointIndex];
}

void CPathGenarateComponent::updateRays(int vCameraIndex,
                                        glm::vec3 vNewPosition) {
  for (int i = 0; i < obsRays.size(); i++) {
    bool visible = false;
    glm::vec3 direction = proxyPoint->vertices[i].Position - vNewPosition;

    visible = bvhTree->strongVisible(vNewPosition,
                                     cameraVertexVector[vCameraIndex].Normal,
                                     proxyPoint->vertices[i].Position, DMAX);

    initRaysMutex.lock();
    {
      for (size_t j = 0; j < obsRays[i].size(); ++j) {
        if (obsRays[i][j].first.w == vCameraIndex) {
          obsRays[i].erase(obsRays[i].begin() + j);
        }
      }
      if (visible) {
        obsRays[i].push_back(std::make_pair(
            glm::vec4(vNewPosition - proxyPoint->vertices[i].Position,
                      vCameraIndex),
            glm::vec4(-glm::normalize(vNewPosition), 1)));
      }
    }
    initRaysMutex.unlock();
  }
}

glm::vec3 CPathGenarateComponent::optimizeOrientation(glm::vec3 vCameraPosition,
                                                      int vCameraIndex) {
  float bestYaw, bestPitch;
  glm::vec3 bestDirection;
  float bestScore = -2;

  std::pair<float, glm::vec3> result = parallel_reduce(
      tbb::blocked_range2d<size_t>(0, 9, 1, 0, 36, 6),
      std::make_pair<float, glm::vec3>(0.f, glm::vec3(0.f)),
      [=](const tbb::blocked_range2d<size_t>& r,
          std::pair<float, glm::vec3> value) -> std::pair<float, glm::vec3> {
        for (size_t i = r.rows().begin(); i != r.rows().end(); ++i) {
          for (size_t j = r.cols().begin(); j != r.cols().end(); ++j) {
            float pitch = 10 * i;
            float yaw = 10 * j;
            float directionZ = sin(glm::radians(pitch));
            float horizenElement = cos(glm::radians(pitch));
            glm::vec3 direction(horizenElement * cos(glm::radians(yaw)),
                                horizenElement * sin(glm::radians(yaw)),
                                -directionZ);
            if (glm::length(direction) - 1.0f > 1e-5) throw "lalala";
            float score = func(vCameraPosition, direction, vCameraIndex);
            if (score > value.first) {
              value.first = score;
              value.second = direction;
            }
          }
        }
        return value;
      },
      [](std::pair<float, glm::vec3> t1,
         std::pair<float, glm::vec3> t2) -> std::pair<float, glm::vec3> {
        if (t1.first > t2.first)
          return t1;
        else
          return t2;
      });

  // for (float iPitch = 0; iPitch < 90; iPitch+=10)
  //{
  //
  //	for (float iYaw = 0; iYaw < 360; iYaw+=10)
  //	{
  //		float directionZ = sin(glm::radians(iPitch));
  //		float horizenElement = cos(glm::radians(iPitch));
  //		glm::vec3 direction(horizenElement*cos(glm::radians(iYaw))
  //			, horizenElement*sin(glm::radians(iYaw)), -directionZ);
  //		if (glm::length(direction) - 1.0f > 1e-5)
  //			throw "lalala";
  //		float score = func(vCameraPosition, direction, vCameraIndex);
  //		if (score > bestScore) {
  //			bestScore = score;
  //			bestYaw = iYaw;
  //			bestPitch = iPitch;
  //			bestDirection = direction;
  //		}
  //	}
  //}
  // if (result.first != bestScore)
  //	throw "lalala";
  if (bestScore == 0) bestDirection = -vCameraPosition;
  return result.second;
}

std::pair<glm::vec3, float> CPathGenarateComponent::downhillSimplex(
    std::vector<glm::vec3>* solution, size_t vCameraIndex) {
  // Evaluate the 4 candidate
  float myValues[4];
  glm::vec3 cameraDirection =
      optimizeOrientation(solution->at(0), vCameraIndex);
  for (size_t i = 0; i < solution->size(); i++) {
    myValues[i] = func(solution->at(i), cameraDirection, vCameraIndex);
  }

  std::size_t best_idx = 0;
  std::size_t lousy_idx = 0;
  std::size_t worst_idx = 0;
  for (size_t i = 1; i < 4; i++) {
    if (myValues[i] > myValues[best_idx]) best_idx = i;
    if (myValues[i] < myValues[worst_idx]) worst_idx = i;
    if (myValues[i] > myValues[worst_idx] && myValues[i] < myValues[lousy_idx])
      lousy_idx = i;
  }

  float best_value = myValues[best_idx];
  float lousy_value = myValues[lousy_idx];
  float worst_value = myValues[worst_idx];

  glm::vec3 bestVertex = solution->at(best_idx);

  glm::vec3 medianPosition(0.0f);
  for (std::size_t k = 0; k < 4 - 1; ++k) {
    if (k != worst_idx) medianPosition += solution->at(k);
  }
  medianPosition /= (4 - 1);

  glm::vec3 refl = medianPosition + (medianPosition - solution->at(worst_idx));

  // float refl_value = func(refl, optimizeOrientation(refl, vCameraIndex),
  // vCameraIndex);
  float refl_value = func(refl, cameraDirection, vCameraIndex);

  if (refl_value > best_value) {
    glm::vec3 exp = refl + (refl - medianPosition);
    // float exp_value = func(exp, optimizeOrientation(exp, vCameraIndex),
    // vCameraIndex);
    float exp_value = func(exp, cameraDirection, vCameraIndex);

    if (exp_value > best_value) {
      /* Expansion */
      solution->at(worst_idx) = exp;
      return {exp, exp_value};
    } else {
      /* Reflection */
      solution->at(worst_idx) = refl;
      return {refl, refl_value};
    }
  } else {
    if (refl_value > worst_value) {
      if (refl_value > lousy_value) {
        /* Reflection */
        solution->at(worst_idx) = refl;
        return {refl, best_value};
      } else {
        /* Outside contraction */
        glm::vec3 con =
            medianPosition + (medianPosition - solution->at(worst_idx)) * 0.5f;
        // float con_value = func(con, optimizeOrientation(con,vCameraIndex),
        // vCameraIndex);
        float con_value = func(con, cameraDirection, vCameraIndex);

        if (con_value > worst_value) {
          solution->at(worst_idx) = con;
          if (con_value > best_value) {
            return {con, con_value};
          } else {
            return {bestVertex, best_value};
          }
        } else {
          /* Shrink */
          shrink(solution, solution->at(best_idx));
          return {bestVertex, best_value};
        }
      }
    } else {
      /* Inside contraction */
      glm::vec3 con =
          medianPosition - (medianPosition - solution->at(worst_idx)) * 0.5f;
      float con_value = func(con, cameraDirection, vCameraIndex);

      if (con_value > worst_value) {
        solution->at(worst_idx) = con;
        if (con_value > best_value) {
          return {con, con_value};
        } else {
          return {bestVertex, best_value};
        }
      } else {
        /* Shrink */
        shrink(solution, solution->at(best_idx));
        return {bestVertex, best_value};
      }
    }
  }
}

void CPathGenarateComponent::optimize(size_t vCameraIndex, int vIter) {
  size_t iCameraIndex = vCameraIndex;
  // this->waitForStepSignal();
  cameraAdjustMesh->changeColor(glm::vec3(0.3, 0.4, 0.5), iCameraIndex);
  // Visualize the candidate
  for (int m = 0; m < simplexes[iCameraIndex].size(); ++m)
    cameraCandidateMesh->changePos(simplexes[iCameraIndex][m],
                                   4 * iCameraIndex + m);

  // Optimize, if downhill failed(return 0), increase scale and reinitialize
  std::pair<glm::vec3, float> optimizeResult =
      downhillSimplex(&(simplexes[iCameraIndex]), iCameraIndex);
  if (optimizeResult.second <= 0 ||
      optimizeResult.first == simplexes[iCameraIndex][0]) {
    if (5 <= cameraInitializeTimes[iCameraIndex]) {
      cameraInitializeTimes[iCameraIndex] = 5;
    }
    cameraInitializeTimes[iCameraIndex] += 1;
    initializeSimplexes(
        SIMPLEXINITIALIZESCALE * cameraInitializeTimes[iCameraIndex],
        cameraVertexVector[iCameraIndex].Position, simplexes[iCameraIndex],
        iCameraIndex);

    /*srand(time(0));
    if (ranks.size() == 0) return;
    std::uniform_int_distribution<> dist(0, ranks[ranks.size() / 2]);
    std::random_device rd;
    std::mt19937 gen(rd());
    size_t magicIndex = ranks[dist(gen)];
    size_t magicIter = 0;

    glm::vec3 newPosition = proxyPoint->vertices[magicIndex].Position
            + (2 * DMIN +
    1)*glm::normalize(proxyPoint->vertices[magicIndex].Normal);

    int y = (newPosition[1] - proxyPoint->bounds.pMin[1]) /
    (proxyPoint->bounds.pMax[1] - proxyPoint->bounds.pMin[1]) * heightMapHeight;
    int x = (newPosition[0] - proxyPoint->bounds.pMin[0]) /
    (proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0]) * heightMapWidth;
    x = (x >= heightMapWidth ? heightMapWidth - 1 : x);
    y = (y >= heightMapHeight ? heightMapWidth - 1 : y);
    if (newPosition[2] < airspace.data[heightMapWidth*y + x]
            || newPosition[2] < DMIN) {
            newPosition[2] += airspace.data[heightMapWidth*y + x] + DMIN;
    }


    cameraVertexVector[iCameraIndex].Position = newPosition;
    cameraVertexVector[iCameraIndex].Normal = glm::normalize(-newPosition);
    cameraAdjustMesh->changeVertex(Vertex(cameraVertexVector[iCameraIndex].Position
            , cameraVertexVector[iCameraIndex].Normal
            , glm::vec3(1.0f, 0.0f, 0.0f)), iCameraIndex);*/

  } else {
    cameraInitializeTimes[iCameraIndex] = 1;
    cameraVertexVector[iCameraIndex].Position = optimizeResult.first;
    cameraVertexVector[iCameraIndex].Normal = optimizeOrientation(
        cameraVertexVector[iCameraIndex].Position, iCameraIndex);

    cameraAdjustMesh->changeVertex(
        Vertex(optimizeResult.first, cameraVertexVector[iCameraIndex].Normal,
               glm::vec3(0.0f, 1.0f, 0.0f)),
        iCameraIndex);
  }
  for (int m = 0; m < simplexes[iCameraIndex].size(); ++m)
    cameraCandidateMesh->changePos(glm::vec3(0), 4 * iCameraIndex + m);
  updateRays(iCameraIndex, optimizeResult.first);
  // if (0 == iCameraIndex)
  //	visualizeCamera(iCameraIndex);
}

void CPathGenarateComponent::visulizeScoreResult() {
  std::cout << "start visualize score" << endl;
  float maxScore = 5.0f;
  float minScore = 0;
  for (size_t i = 0; i < obsRays.size(); i++) {
    proxyPoint->changeColor(
        glm::vec3(min(reconstructionScore[i], targetReconstrucbility) /
                  targetReconstrucbility),
        i);
  }
  std::cout << "done visualize score" << endl;
}

void CPathGenarateComponent::optimize_nadir() {
  tbb::task_scheduler_init init();
  // update the obs rays
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](const size_t vi) { initRays(vi); });
  // Initialize Simplexes
  srand(std::time(nullptr));
  simplexes.resize(cameraVertexVector.size());
  for (size_t i = 0; i < cameraVertexVector.size(); ++i) {
    initializeSimplexes(SIMPLEXINITIALIZESCALE, cameraVertexVector[i].Position,
                        simplexes[i], i);
  }
  cameraInitializeTimes.resize(cameraVertexVector.size(), 1);

  // Start iter
  std::uniform_int_distribution<> dist(0, cameraVertexVector.size() - 1);
  std::random_device rd;
  std::mt19937 gen(rd());
  for (int iter = 0; iter < 100000; ++iter) {
    vector<size_t> targetCameraIndices;
    srand(std::time(nullptr));
    // Select independent view to optimize parallel
    for (int iCameraIndex = 0; iCameraIndex < cameraVertexVector.size() * 2;
         ++iCameraIndex) {
      bool toClose = true;
      size_t nowCameraIndex = dist(gen);
      for (auto& iiTargetCamera : targetCameraIndices) {
        if (glm::length(cameraVertexVector[iiTargetCamera].Position -
                        cameraVertexVector[nowCameraIndex].Position) < DMAX) {
          toClose = false;
          break;
        }
      }
      if (toClose) targetCameraIndices.push_back(nowCameraIndex);
    }

    // Downhill simplex
    tbb::parallel_for(size_t(0), targetCameraIndices.size(), [=](size_t i) {
      optimize(targetCameraIndices[i], iter);
    });

    tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                      [=](size_t i) { evalueteReconstrucbility(i); });

    // Get the result
    visulizeScoreResult();
    averageScore = 0;
    for (auto& p : reconstructionScore) {
      averageScore += p;
    }
    averageScore /= proxyPoint->vertices.size();
    targetReconstrucbility = 2 * averageScore;
    for (auto& p : obsRays) {
      averageSeenSample =
          p.size() > averageSeenSample ? p.size() : averageSeenSample;
    }
    averageSeenSample = averageSeenSample / 2;
    std::cout << "Average Score in " << iter << " : " << averageScore << endl
              << endl;
    // Sort the reconstructibility
    ranks.resize(obsRays.size());
    std::iota(ranks.begin(), ranks.end(), 0);
    std::sort(ranks.begin(), ranks.end(), [&](std::size_t a, std::size_t b) {
      return reconstructionScore[a] < reconstructionScore[b];
    });

    reconstructionScore.resize(proxyPoint->vertices.size(), 0);

    int photoID = 0;
    fileFp.open(fileLogPath, ios::out);
    char c[8];
    sprintf(c, "%05d", photoID);
    string photoName = string(c) + ".jpeg,";
    for (size_t iCameraIndex = 0; iCameraIndex < cameraVertexVector.size();
         iCameraIndex++) {
      fileFp << photoName << cameraVertexVector[iCameraIndex].Position.x << ","
             << cameraVertexVector[iCameraIndex].Position.y << ","
             << cameraVertexVector[iCameraIndex].Position.z << ","
             << cameraVertexVector[iCameraIndex].Normal.x << ","
             << cameraVertexVector[iCameraIndex].Normal.y << ","
             << cameraVertexVector[iCameraIndex].Normal.z << endl;
      assert(cameraVertexVector[iCameraIndex].Position.z > 0);
      photoID += 1;
      char s[8];
      sprintf(s, "%05d", photoID);
      photoName = string(s) + ".jpeg,";
    }
    // fileFp.flush();
    fileFp.close();
  }
}

void CPathGenarateComponent::simplexPoint() {
  obsRays.resize(proxyPoint->vertices.size());

  bvhTree = new ACCEL::BVHAccel(this->m_Scene->m_Models.at("proxy_mesh"));

  // update the obs rays
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](size_t i) { initRays(i); });

  vector<Vertex> outVertexes;
  for (int i = 0; i < obsRays.size(); ++i) {
    if (obsRays[i].size() != 0) {
      outVertexes.push_back(proxyPoint->vertices[i]);
    }
  }
  CMesh* outMesh = new CPointCloudMesh(outVertexes);
  CMesh::saveMesh(
      outMesh,
      "C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point_simplex.ply");
}

void CPathGenarateComponent::visualizeExistCamera() {
  ifstream ifs(fileLogPath, ios::in);
  string line;
  getline(ifs, line);
  size_t splitPos;
  size_t psplitPos;
  while (line.length() > 5) {
    splitPos = line.find(",", 0);
    float x, y, z, dx, dy, dz;
    std::vector<float> splitItems = LiteUtil::splitString<float>(line, ",");
    x = splitItems[1];
    y = splitItems[2];
    z = splitItems[3];
    dx = splitItems[4];
    dy = splitItems[5];
    dz = splitItems[6];
    Vertex v;
    v.Position = glm::vec3(x, y, z);
    v.Normal = glm::vec3(dx, dy, dz);
    cameraVertexVector.push_back(v);
    getline(ifs, line);
  }
  cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.3f, 0.7f, 1.f), 20);
  cameraMesh->isRender = true;
  cameraMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));

  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](const size_t vi) { initRays(vi); });

  std::cout << "start visualize" << endl;
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](size_t i) { evalueteReconstrucbility(i); });

  // Get the result
  visulizeScoreResult();
  std::cout << "done visualize" << endl;

  // size_t cameraIndex = 4;
  // std::cout << "start visualize" << endl;
  // cameraMesh->changeColor(glm::vec3(0, 0.5, 0.5), cameraIndex);
  // for (int i = 0; i < proxyPoint->vertices.size(); ++i)
  //{
  //	if (strongVisible(cameraVertexVector[cameraIndex].Position
  //		, cameraVertexVector[cameraIndex].Normal,
  // proxyPoint->vertices[i].Position 		, bvhTree, DMAX))
  //	{
  //		proxyPoint->changeColor(glm::vec3(0.0f, 0.0f, 1.0f), i);
  //		std::cout << "change " << i << " point color to blue" << endl;
  //	}
  //}
  // std::cout << "done visualize" << endl;
  // cameraMesh->changeColor(glm::vec3(0, 1, 0), cameraIndex);
}

void CPathGenarateComponent::visualizeSurroundingsViewCamera() {
  std::uniform_real_distribution<float> ud(0, 1);
  std::mt19937 rd;
  for (const auto& point : proxyPoint->vertices) {
    glm::vec3 direction;
    if (point.Normal[0] > point.Normal[1])
      if (point.Normal[0] > point.Normal[2])
        direction =
            (point.Normal[0] > 0 ? 1.f : -1.f) * glm::vec3(20.f, 0.f, 0.f);
      else
        direction = glm::vec3(0.f, 0.f, 20.f);
    else if (point.Normal[1] > point.Normal[2])
      direction =
          (point.Normal[1] > 0 ? 1.f : -1.f) * glm::vec3(0.f, 20.f, 0.f);
    else
      direction = glm::vec3(0.f, 0.f, 20.f);
    Vertex v;
    v.Position = point.Position + direction;
    v.Normal = -point.Normal;
    if (ud(rd) < 0.1f) cameraVertexVector.push_back(v);
  }
  cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.3f, 0.4f, 0.6f), 30);
  cameraMesh->isRender = true;
  cameraMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));

  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](const size_t vi) { initRays(vi); });

  std::cout << "start visualize" << endl;
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](size_t i) { evalueteReconstrucbility(i); });

  // Get the result
  visulizeScoreResult();
  std::cout << "done visualize" << endl;
}

void CPathGenarateComponent::visualizeRandomViewCamera() {
  std::uniform_real_distribution<float> ud(0, 1);
  std::mt19937 rd;

  float bounds = 40.f;
  glm::vec3 diff =
      proxyPoint->bounds.pMax - proxyPoint->bounds.pMin + 2 * bounds;

  for (size_t index = 0; index < 500; index++) {
    float x = ud(rd) * diff[0] + proxyPoint->bounds.pMin[0] - bounds;
    float y = ud(rd) * diff[1] + proxyPoint->bounds.pMin[1] - bounds;
    float z = ud(rd) * diff[2] + proxyPoint->bounds.pMin[2];

    Vertex v;
    v.Position = glm::vec3(x, y, z);
    v.Normal = -v.Position;
    cameraVertexVector.push_back(v);
  }
  cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.3f, 0.4f, 0.6f), 30);
  cameraMesh->isRender = true;
  cameraMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));

  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](const size_t vi) { initRays(vi); });

  std::cout << "start visualize" << endl;
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](size_t i) { evalueteReconstrucbility(i); });

  // Get the result
  visulizeScoreResult();
  std::cout << "done visualize" << endl;
}

void CPathGenarateComponent::visualizeAsiaCamera() {
  ifstream ifs("../../../my_test/asia_ny.csv", ios::in);
  string line;
  getline(ifs, line);
  size_t splitPos;
  size_t psplitPos;
  while (line.length() > 5) {
    splitPos = line.find(",", 0);
    float x, y, z, dx, dy, dz;
    std::vector<float> splitItems = LiteUtil::splitString<float>(line, ",");
    x = splitItems[0];
    y = splitItems[1];
    z = splitItems[2];
    dx = splitItems[3];
    dy = splitItems[4];
    dz = splitItems[5];
    Vertex v;
    v.Position = glm::vec3(x, y, z);
    v.Normal = glm::vec3(dx, dy, dz);
    cameraVertexVector.push_back(v);
    getline(ifs, line);
  }
  cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.3f, 0.7f, 1.f), 30);
  cameraMesh->isRender = true;
  cameraMesh->isRenderNormal = true;
  // Lock the target arrays
  std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
  CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));

  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](const size_t vi) { initRays(vi); });

  std::cout << "start visualize" << endl;
  tbb::parallel_for(size_t(0), proxyPoint->vertices.size(),
                    [=](size_t i) { evalueteReconstrucbility(i); });

  // Get the result
  visulizeScoreResult();
  std::cout << "done visualize" << endl;
}

void CPathGenarateComponent::staticsForPath(float reconThresh) {
  size_t cover = 0, recon = 0;
  for (auto item : reconstructionScore) {
    if (item > reconThresh) recon += 1;
  }
  for (auto item : obsRays) {
    if (item.size() > 1) cover += 1;
  }

  std::cout << "Cover ratio: "
            << static_cast<float>(cover) / static_cast<float>(obsRays.size())
            << std::endl;

  std::cout << "Recon(>" << reconThresh << ") ratio: "
            << static_cast<float>(recon) /
                   static_cast<float>(reconstructionScore.size())
            << std::endl;
}

std::ostream& operator<<(std::ostream& os, const float4& x) {
  os << x.x << ", " << x.y;
  return os;
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
  LiteS_Trajectory::generateNadir(glm::vec3(70, 70, 30), glm::vec3(-70, -70, 0),
                                  90.f, 0.8, cameraVertexVector);
  CMesh* cameraMesh =
      new CPointCloudMesh(cameraVertexVector, glm::vec3(0.f, 1.f, 0.f), 10.f);
  cameraMesh->isRenderNormal = true;
   std::unique_lock<std::mutex> ul(CEngine::m_addMeshMutex);
   CEngine::toAddModels.push_back(std::make_pair("camera", cameraMesh));
   ul.unlock();

   tbb::task_scheduler_init init(1); 
  tbb::atomic<size_t> countUpdate = 0;
  // 2 Update rays and evaluate re constructibility
  tbb::parallel_for(
      tbb::blocked_range<size_t>(size_t(0), cameraVertexVector.size()),
      [&](tbb::blocked_range<size_t>& r) {
        cudaStream_t stream;
        CUDACHECKERROR(cudaStreamCreate(&stream));

        cudaEvent_t event;
        CUDACHECKERROR(cudaEventCreateWithFlags(
            &event, cudaEventDefault | cudaEventDisableTiming));

        for (size_t i = r.begin(); i < r.end(); i++) {
          updateObsRays(
              grid, block, stream, dBVHPointer, dPointsPtr, numPoints,
              myObsRays.data,
              CCDataStructure::glmToFloat3(cameraVertexVector[i].Position),
              CCDataStructure::glmToFloat3(cameraVertexVector[i].Normal));
          CUDACHECKERROR(cudaEventRecord(event, stream));
          // CUDACHECKERROR(cudaEventQuery(event));
          cudaEventSynchronize(event);
        }
        countUpdate.fetch_and_add(r.end() - r.begin());
        std::cout << countUpdate << "/" << cameraVertexVector.size()
                  << "			";
        // CUDACHECKERROR(cudaThreadSynchronize());
      },
      tbb::simple_partitioner());

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
