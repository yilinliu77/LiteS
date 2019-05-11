
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
BVHACCEL::BVHAccel* bvhTree;

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

void CPathGenarateComponent::extraAlgorithm() {
	CMesh* pointCloud = this->m_Scene->m_Models.at("point")->meshes[0];
	CModel* meshModel = this->m_Scene->m_Models.at("gt_mesh");
	BVHACCEL::BVHAccel bvhTree(meshModel->meshes);
	const glm::vec3 endPos;

	const glm::vec3& boundsMax = pointCloud->bounds.pMax;
	const glm::vec3& boundsMin = pointCloud->bounds.pMin;
	const size_t numPoint = pointCloud->vertices.size();

	std::vector<Vertex> outVertex;
	std::vector<bool> pointSeen(numPoint, false);

	const float height = boundsMax[2] + 10.f;
	std::vector<glm::vec3> cameraPos;
	float step = 30.f;
	for (float x = boundsMin[0] - 60.f; x < boundsMax[0] + 60.f; x += step) {
		for (float y = boundsMin[1] - 60.f; y < boundsMax[1] + 60.f; y += step) {
			cameraPos.push_back(glm::vec3(x, y, height));
		}
	}
	size_t cameraIndex = 0;
	for (auto& itemCamera : cameraPos) {
		tbb::parallel_for(tbb::blocked_range<size_t>(0, numPoint)
			, [&](const tbb::blocked_range<size_t> & r) {
				for (size_t i = r.begin(); i != r.end(); ++i) {
					const glm::vec3& pointPos = pointCloud->vertices[i].Position;
					Ray ray(itemCamera, glm::normalize(pointPos - itemCamera));
					SurfaceInteraction sr;
					if (bvhTree.Intersect(ray, &sr)) {
						if (glm::length(sr.pHit - pointPos) < 1e-1f)
							pointSeen[i] = true;
					}
					else
						pointSeen[i] = true;
				}
			}
		);
		std::cout << cameraIndex << "/" << cameraPos.size() << std::endl;
		cameraIndex += 1;
	}


	for (size_t i = 0; i < pointCloud->vertices.size(); ++i) {
		if (pointSeen[i])
			outVertex.push_back(pointCloud->vertices[i]);
	}
	CMesh* outMesh = new CPointCloudMesh(outVertex);
	outMesh->saveMesh(outMesh, "../../../my_test/inside_cleaned.ply");

	std::cout << "Extra Algorithm done" << endl;

	this->waitForStepSignal();
	return;
}

void CPathGenarateComponent::extraInit() {}
