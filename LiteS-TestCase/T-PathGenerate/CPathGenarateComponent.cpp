#include "CPathGenarateComponent.h"
#include "CPointCloudMesh.h"
#include <CEngine.h>
#include "CBVHACCEL.h"
#include <algorithm>
#include <numeric>
#include "util.h"
#include <set>
#include <random>
#include <array>
#include<fstream>

#include <glm/gtc/matrix_access.hpp>

const float DMAX = 35;
const float DMIN = 5;
const float SIMPLEXINITIALIZESCALE = 3.0f;

CMesh* proxyPoint;
CModel* proxyModel;
BVHAccel* bvhTree;
CMesh* cameraPosMesh;
vector<Vertex> cameraPosVector;
vector<Vertex> cameraAdjustVector;

vector<float> reconstructionScore;

vector<size_t> cameraInitializeTimes;

std::mutex eraseMutex;

const int heightMapWidth = 32;
const int heightMapHeight = 32;
My8BitRGBImage airspace;

std::ofstream fileFp;

//Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y, Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;
vector<vector<int>> seenSample;

CPathGenarateComponent::CPathGenarateComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::generate_nadir() {
	proxyPoint = DisplayPass->m_Models[0]->meshes[0];

	glm::vec3 mesh_dim = proxyPoint->bounds.pMax - proxyPoint->bounds.pMin;
	float max_height = proxyPoint->bounds.pMax[2] + 10;

	float step = 15;

	glm::vec3 cameraPos = proxyPoint->bounds.pMin;
	cameraPos.z = max_height;
	while (cameraPos.x< proxyPoint->bounds.pMax.x)
	{
		while (cameraPos.y < proxyPoint->bounds.pMax.y)
		{
			Vertex v;
			v.Position = cameraPos;
			v.Normal = glm::normalize(-cameraPos);
			cameraPosVector.push_back(v);
			cameraPos.y += step;
		}
		cameraPos.x += step;
		cameraPos.y = proxyPoint->bounds.pMin.y;

	}

	cameraPosMesh = new CPointCloudMesh(cameraPosVector,glm::vec3(1.0f,0.0f,0.0f),30);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraPosMesh);
}

void initRays(int vi) {
	glm::vec3 samplePos = proxyPoint->vertices[vi].Position;
	glm::vec3 sampleNormal = proxyPoint->vertices[vi].Normal;

	//Used for OpenMP
	vector<std::pair<glm::vec4, glm::vec4>> localObsRays;
	vector<int> localSeenSample;

	for (int j=0;j<cameraPosVector.size();++j)
	{
		glm::vec3 direction = cameraPosVector[j].Position - samplePos;
		//
		// Inside the frustum
		//
		// 0.087f ~ cos(85.0f / 180.0f * pi)
		float viewAngleCos = glm::dot(-glm::normalize(direction), -glm::normalize(cameraPosVector[j].Position));
		if(viewAngleCos <0.7 && viewAngleCos>0)
			continue;
		
		//
		// Not occluded
		//
		if (bvhTree->Visible(cameraPosVector[j], proxyPoint->vertices[vi],1.0f))
		{
			localObsRays.push_back(std::make_pair(glm::vec4(direction,j), glm::vec4(-glm::normalize(cameraPosVector[j].Position),1)));
			//seenSample[j].push_back(vi);
		}

		
	}

	#pragma omp critical
	{
		obsRays[vi].insert(obsRays[vi].end(), localObsRays.begin(), localObsRays.end());
		//seenSample.insert(seenSample.end(), localSeenSample.begin(), localSeenSample.end());
	}
	
}

void updateRays(int vCameraIndex, glm::vec3 vNewPosition) {
	for (int i = 0; i < obsRays.size(); i++)
	{
		bool visible = false;
		glm::vec3 direction = proxyPoint->vertices[i].Position - vNewPosition;
		float viewAngleCos = glm::dot(glm::normalize(direction), -glm::normalize(vNewPosition));
		if (viewAngleCos > 0.7)
			visible = true;

		if (visible) {
			Vertex t;
			t.Position = vNewPosition;
			visible = bvhTree->Visible(t, proxyPoint->vertices[i], 1.0f);
		}
		#pragma omp critical
		{
			for (size_t j = 0; j < obsRays[i].size(); ++j)
			{
				if (obsRays[i][j].first.w == vCameraIndex)
				{
					obsRays[i].erase(obsRays[i].begin() + j);
				}
			}
			if (visible) {
				obsRays[i].push_back(std::make_pair(glm::vec4(vNewPosition - proxyPoint->vertices[i].Position, vCameraIndex)
					, glm::vec4(-glm::normalize(vNewPosition), 1)));
			}
		}
	}
}

void optimizeOrientation(int vCameraIndex) {
	float bestYaw, bestPitch;
	for (size_t pitch = 0; pitch < 90; pitch+=10)
	{
		for (size_t yaw = 0; yaw < 360; yaw+=30)
		{

		}
	}
}

void CPathGenarateComponent::optimize_nadir() {
	cameraAdjustVector = vector<Vertex>(cameraPosVector);
	obsRays.resize(proxyPoint->vertices.size());
	
	seenSample.resize(cameraPosVector.size());
	reconstructionScore.resize(proxyPoint->vertices.size(),0);
	vector<CMesh*> meshVector;
	meshVector.push_back(proxyPoint);

	//proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	proxyModel = new CModel("C:/repos/GRAPHICS/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

	//
	// Generate height map as free airspace
	//
	airspace.ncols = heightMapWidth;
	airspace.nrows = heightMapHeight;
	airspace.data = new float[heightMapWidth*heightMapHeight];
	for (size_t i=0;i<heightMapHeight*heightMapWidth;++i)
	{
		airspace.data[i] = -9999999;
	}

	float xDim = proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0];
	float yDim = proxyPoint->bounds.pMax[1] - proxyPoint->bounds.pMin[1];

	for (size_t i = 0; i < proxyPoint->vertices.size(); ++i)
	{
		int x = static_cast<int>((proxyPoint->vertices[i].Position.x - proxyPoint->bounds.pMin[0])/ xDim * heightMapWidth);
		int y = static_cast<int>((proxyPoint->vertices[i].Position.y - proxyPoint->bounds.pMin[1]) /yDim*heightMapHeight);
		if (airspace.data[y*heightMapWidth + x] < proxyPoint->vertices[i].Position.z)
			airspace.data[y*heightMapWidth + x] = proxyPoint->vertices[i].Position.z;
	}

	/*LinearBVHNode* nodes = bvhTree->getLinearNodes();
	vector<Tri*>& meshes = bvhTree->getOrderedTriangles();
	
	for (size_t i = 0; i < meshes.size(); ++i)
	{
		glm::vec3 v1 = meshes[i]->v1.Position;
		glm::vec3 v2 = meshes[i]->v2.Position;
		glm::vec3 v3 = meshes[i]->v3.Position;
		int y = (v1[1] - nodes->bounds.pMin[1]) / yDim * heightMapHeight;
		int x = (v1[0] - nodes->bounds.pMin[0]) / xDim * heightMapWidth;
		if (airspace.data[y*heightMapWidth + x] < v1[2])
		{
			airspace.data[y*heightMapWidth + x] = v1[2];
		}
		y = (v2[1] - nodes->bounds.pMin[1]) / yDim * heightMapHeight;
		x = (v2[0] - nodes->bounds.pMin[0]) / xDim * heightMapWidth;
		if (airspace.data[y*heightMapWidth + x] < v2[2])
		{
			airspace.data[y*heightMapWidth + x] = v2[2];
		}
		y = (v3[1] - nodes->bounds.pMin[1]) / yDim * heightMapHeight;
		x = (v3[0] - nodes->bounds.pMin[0]) / xDim * heightMapWidth;
		if (airspace.data[y*heightMapWidth + x] < v3[2])
		{
			airspace.data[y*heightMapWidth + x] = v3[2];
		}
	}*/

	//update the obs rays
	#pragma omp parallel for
	for (int i = 0; i < proxyPoint->vertices.size(); ++i) {
		initRays(i);
	}

	//
	// Initialize Simplexes
	//
	std::vector<std::vector<glm::vec3>> simplexes;
	simplexes.resize(cameraPosVector.size());
	for (size_t i = 0; i < cameraPosVector.size(); ++i)
	{
		initializeSimplexes(SIMPLEXINITIALIZESCALE, cameraPosVector[i].Position, simplexes[i]);
	}
	cameraInitializeTimes.resize(cameraPosVector.size(), 1);

	//Camera Pos, Camera Direction, Camera Index
	std::function<float(glm::vec3, glm::vec3, size_t)> func =
		[&](glm::vec3 const & vCameraPos, glm::vec3 vCameraDirection, size_t vCameraIndex) -> float
	{
		//
		// Return -1 if to close to mesh or ground
		//
		if (vCameraPos.z < DMIN || bvhTree->KNearest(vCameraPos).first<DMIN)
			return -1;

		//
		// In airspace
		//
		int y = (vCameraPos[1] - proxyPoint->bounds.pMin[1]) / yDim * heightMapHeight;
		int x = (vCameraPos[0] - proxyPoint->bounds.pMin[0]) / xDim * heightMapWidth;
		if (vCameraPos[2]<airspace.data[heightMapWidth*y+x])
		{
			return -1;
		}

		float totalScore = 0;
		for (size_t pointIndex = 0; pointIndex < proxyPoint->vertices.size(); pointIndex++)
		{
			//
			// Break if point invisible
			//
			glm::vec3 samplePosition = proxyPoint->vertices[pointIndex].Position;
			bool visible = false;
			//for (auto & cameraItem : obsRays[pointIndex]) {
			//	if (cameraItem.first.w == vCameraIndex) {
			//		visible = true;
			//		break;
			//	}
			//}
			Vertex t;
			t.Position = vCameraPos;
			visible = bvhTree->Visible(t, proxyPoint->vertices[pointIndex], 1.0f);
			if (!visible) continue;

			glm::vec3 sampleNormal = proxyPoint->vertices[pointIndex].Normal;
			glm::vec3 sample2TargetCamera = vCameraPos - samplePosition;

			//
			// Break Inside the frustum
			//
			if (glm::length(samplePosition - vCameraPos) > DMAX)
				continue;
			// 0.087f ~ cos(85.0f / 180.0f * pi)
			float viewAngleCos = glm::dot(-glm::normalize(sample2TargetCamera), -glm::normalize(vCameraPos));
			if (viewAngleCos < 0.7 && viewAngleCos>0)
				continue;

			float score = 0;
			int foundReference = 0;

			for (auto & cameraItem : obsRays[pointIndex]) {
				if (cameraItem.first.w == vCameraIndex) {
					foundReference = 1;
					continue;
				}

				glm::vec3 camera1Pos = cameraPosVector[cameraItem.first.w].Position;
				glm::vec3 camera1Direction = cameraPosVector[cameraItem.first.w].Normal;
				glm::vec3 sample2Camera1 = camera1Pos - samplePosition;

				float camera1Distance = glm::length(sample2Camera1);
				float camera2Distance = glm::length(sample2TargetCamera);
				float maxDistance = std::max(camera1Distance, camera2Distance);

				if(maxDistance>DMAX)
					continue;

				float viewAngleCos = glm::dot(glm::normalize(sample2Camera1), glm::normalize(sample2TargetCamera));
				float viewAngle = std::acos(viewAngleCos);

				

				// w1 = 1 / ( 1 + exp( -k1 * ( alpha - alpha1 ) ) )
				float w1 = 1 / (1 + (std::exp(-32 * (viewAngle - glm::pi<float>() / 16))));

				// w2 = min( max(d1, d2) / dmax, 1 )
				float w2 = 1 - std::min(maxDistance / DMAX, 1.0f);

				// w3 =1 - 1 / ( 1 + exp( -k3 * ( alpha - alpha3 ) ) )
				float w3 = 1 - 1 / (1 + std::exp(-8 * (viewAngle - glm::pi<float>() / 4)));

				glm::vec3 viewPlaneNormal = glm::normalize(glm::cross(sample2Camera1, sample2TargetCamera));
				float sinTheta = glm::dot(viewPlaneNormal, sampleNormal);

				score += w1 * w2 * w3 * std::sqrt(1 - sinTheta * sinTheta);

			}

			//reconstructionScore[pointIndex] = score;

			// 1 prevent divide by 0
			// Last element encourage seen the less seen point
			float pointSeen = ((obsRays[pointIndex].size()) > 0 ? (obsRays[pointIndex].size()) : 1);
			totalScore += (score / pointSeen) * (1 / pointSeen);
		}
		return totalScore ;
	};

	//
	// Visualize
	//
	vector<Vertex> cameraCandidate;
	cameraCandidate.resize(cameraPosVector.size() * 4);
	CMesh* cameraCandidateMesh = new CPointCloudMesh(cameraAdjustVector, glm::vec3(0.0f, 0.0f, 1.0f), 10);

	CMesh* cameraAdjustMesh = new CPointCloudMesh(cameraAdjustVector, glm::vec3(0.0f, 1.0f, 0.0f), 20);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraAdjustMesh);
	CEngine::toAddMeshes.push_back(cameraCandidateMesh);
	lg.~lock_guard();
	//
	// Start iter
	//
	for (int iter = 0; iter < 100; ++iter) {
		vector<size_t> targetCameraIndices;
		//
		// Select independent view to optimize parallel
		//
		for (int iCameraIndex=0;iCameraIndex<cameraPosVector.size();++iCameraIndex)
		{
			bool toClose = true;
			for (auto& iiTargetCamera:targetCameraIndices)
			{
				if (glm::length(cameraPosVector[iiTargetCamera].Position 
					- cameraPosVector[iCameraIndex].Position)<-1)
				{
					toClose=false;
					break;
				}
			}
			if(toClose)
				targetCameraIndices.push_back(iCameraIndex);
		}

		//
		// Downhill simplex
		//
		#pragma omp parallel for 
		for (int i = 0; i < targetCameraIndices.size(); i++) {
			size_t iCameraIndex = targetCameraIndices[i];
			//
			// Optimize, if downhill failed(return 0), increase scale and reinitialize
			//
			std::pair<glm::vec3, float> optimizeResult = downhillSimplex(&(simplexes[iCameraIndex]), iCameraIndex, func);
			#pragma omp critical
			{
				if (optimizeResult.second == 0) {
					cameraInitializeTimes[iCameraIndex] += 1;
					initializeSimplexes(SIMPLEXINITIALIZESCALE * cameraInitializeTimes[iCameraIndex]
						, cameraPosVector[iCameraIndex].Position
						, simplexes[iCameraIndex]);
					for (int m = 0; m < simplexes[iCameraIndex].size(); ++m)
					{
						cameraCandidateMesh->changeVertex(simplexes[iCameraIndex][m], 4 * iCameraIndex + m);
					}
				}
				else {
					cameraInitializeTimes[iCameraIndex] = 1;
					cameraPosVector[iCameraIndex].Position = optimizeResult.first;
					cameraAdjustVector[iCameraIndex].Position = optimizeResult.first;
					cameraAdjustMesh->changeVertex(optimizeResult.first, iCameraIndex);

				}
			}
			updateRays(iCameraIndex, optimizeResult.first);

			//for (int pointInObsRays = 0; pointInObsRays < obsRays.size(); ++pointInObsRays) {
			//#pragma omp critical
			//	{
			//	for (int cameraInObsrays = 0; cameraInObsrays < obsRays[pointInObsRays].size(); ++cameraInObsrays) {
			//		
			//			if (obsRays[pointInObsRays][cameraInObsrays].second.w == 0) {
			//				obsRays[pointInObsRays].erase(obsRays[pointInObsRays].begin() + cameraInObsrays);
			//			}
			//		}
			//	}
			//}

			cout << "No." << iter << " iter:" << optimizeResult.second << endl;
		}
		fileFp.open("camera_log", ios::out);
		for (size_t iCameraIndex = 0; iCameraIndex < cameraPosVector.size(); iCameraIndex++) {
			fileFp << cameraPosVector[iCameraIndex].Position.x
				<<","<< cameraPosVector[iCameraIndex].Position.y
				<<","<< cameraPosVector[iCameraIndex].Position.z << endl;
		}
		fileFp.close();
	}

	cameraAdjustMesh->changeColor(glm::vec3(0, 0.5, 0.5), 0);
	for (int i = 0; i < obsRays.size(); ++i)
	{
		for (int j = 0; j < obsRays[i].size(); ++j) {
			if (obsRays[i][j].first.w == 0)
			{
				proxyPoint->changeColor(glm::vec3(0.0f, 0.0f, 1.0f), i);
				cout << "change " << i << " point color to blue" << endl;
			}
			this->waitForStepSignal();
		}

	}

}

void CPathGenarateComponent::simplexPoint() {
	obsRays.resize(proxyPoint->vertices.size());

	proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

	//update the obs rays
	#pragma omp parallel for
	for (int i = 0; i < proxyPoint->vertices.size(); ++i) {
		initRays(i);
	}

	vector<Vertex> outVertexes;
	for (int i = 0; i < obsRays.size(); ++i) {
		if (obsRays[i].size() != 0) {
			outVertexes.push_back(proxyPoint->vertices[i]);
		}
	}
	CMesh *outMesh = new CPointCloudMesh(outVertexes);
	saveMesh(outMesh, "C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point_simplex.ply");
}

void CPathGenarateComponent::extraAlgorithm() {
	generate_nadir();

	//simplexPoint();

	optimize_nadir();


	cout << "Extra Algorithm done" << endl;

	

	this->waitForStepSignal();
	return;
	
}

void CPathGenarateComponent::extraInit() {}


