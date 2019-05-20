
#include "CCalculateAverageDistanceComponent.h"
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

#define THREADSPERBLOCK 512
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

CCalculateAverageComponent::CCalculateAverageComponent(
    const map<string, CPass*>& vPass,
	CScene* vScene,
	const std::string vResourceDir)
	: CPointCloudComponent(vPass, vScene, vResourceDir) {
	DisplayPass = this->m_Pass.at("display");
}

CCalculateAverageComponent::~CCalculateAverageComponent() = default;

void CCalculateAverageComponent::extraAlgorithm() {
  

	std::cout << "Extra Algorithm done" << endl;

	this->waitForStepSignal();
	return;
}

void CCalculateAverageComponent::extraInit() {}
