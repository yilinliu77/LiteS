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
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_scheduler_init.h>
#include <iostream>

#include <glm/gtc/matrix_access.hpp>

const float DMAX = 50;
const float DMIN = 10;
const float SIMPLEXINITIALIZESCALE = 3.0f;
const int heightMapWidth = 32;
const int heightMapHeight = 32;

// Input Mesh
CMesh* proxyPoint;
BVHAccel* bvhTree;

CMesh* cameraMesh;
vector<Vertex> cameraVertexVector;
CMesh* cameraCandidateMesh;

My8BitRGBImage airspace;
tbb::spin_mutex initRaysMutex;
std::ofstream fileFp;

//For Reconstruction
vector<float> reconstructionScore;
std::vector<std::vector<glm::vec3>> simplexes;
vector<size_t> cameraInitializeTimes;
CMesh* cameraAdjustMesh;

//Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y, Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;

CPathGenarateComponent::CPathGenarateComponent(const map<string, CPass*>& vPass, CScene * vScene) :CPointCloudComponent(vPass,vScene){
	DisplayPass = this->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::visualizeCamera(size_t vCameraIndex) {
	cout << "start visualize" << endl;
	cameraAdjustMesh->changeColor(glm::vec3(0, 0.5, 0.5), vCameraIndex);
	for (int i = 0; i < obsRays.size(); ++i)
	{
		for (int j = 0; j < obsRays[i].size(); ++j) {
			if (obsRays[i][j].first.w == vCameraIndex)
			{
				proxyPoint->changeColor(glm::vec3(0.0f, 0.0f, 1.0f), i);
				cout << "change " << i << " point color to blue" << endl;
			}
		}
	}
	cout << "done visualize" << endl;
	this->waitForStepSignal();
	cameraAdjustMesh->changeColor(glm::vec3(0, 1,0), vCameraIndex);
}

void CPathGenarateComponent::generate_nadir() {
	proxyPoint = this->m_Scene->m_Models.at("proxy_point")->meshes[0];

	glm::vec3 mesh_dim = proxyPoint->bounds.pMax - proxyPoint->bounds.pMin;
	float max_height = proxyPoint->bounds.pMax[2] + 20;

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
			cameraVertexVector.push_back(v);
			cameraPos.y += step;
		}
		cameraPos.x += step;
		cameraPos.y = proxyPoint->bounds.pMin.y;

	}

	cameraMesh = new CPointCloudMesh(cameraVertexVector,glm::vec3(1.0f,0.0f,0.0f),30);
	cameraAdjustMesh = new CPointCloudMesh(cameraVertexVector, glm::vec3(0.0f, 1.0f, 0.0f), 30);
	vector<Vertex> cameraCandidate;
	cameraCandidate.resize(cameraVertexVector.size() * 4);
	cameraCandidateMesh = new CPointCloudMesh(cameraCandidate, glm::vec3(0.0f, 0.0f, 1.0f), 10);
	CModel* cameraModel = new CModel;
	cameraModel->isRender = true;
	cameraModel->meshes.push_back(cameraMesh);
	cameraModel->meshes.push_back(cameraAdjustMesh);
	cameraModel->meshes.push_back(cameraCandidateMesh);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddModels.push_back(std::make_pair("camera", cameraModel));
}

//Camera Pos, Camera Direction, Camera Index
std::function<float(glm::vec3, glm::vec3, size_t)> func =
[&](glm::vec3 const & vCameraPos, glm::vec3 vCameraDirection, size_t vCameraIndex) -> float
{
	// Return -1 if to close to mesh or ground
	if (vCameraPos.z < DMIN || bvhTree->KNearest(vCameraPos).first < DMIN)
		return -1;

	// In airspace
	int y = (vCameraPos[1] - proxyPoint->bounds.pMin[1]) / (proxyPoint->bounds.pMax[1] -proxyPoint->bounds.pMin[1]) * heightMapHeight;
	int x = (vCameraPos[0] - proxyPoint->bounds.pMin[0]) / (proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0]) * heightMapWidth;
	x = (x >= heightMapWidth ? heightMapWidth - 1 : x);
	y = (y >= heightMapHeight ? heightMapWidth - 1 : y);
	if (vCameraPos[2] < airspace.data[heightMapWidth*y + x])
	{
		return -1;
	}

	float totalScore = 0;
	for (size_t pointIndex = 0; pointIndex < proxyPoint->vertices.size(); pointIndex++)
	{
		// Break if point invisible
		glm::vec3 samplePosition = proxyPoint->vertices[pointIndex].Position;
		bool visible = false;

		visible = strongVisible(vCameraPos, vCameraDirection, samplePosition,bvhTree,DMAX);
		if (!visible) continue;

		glm::vec3 sampleNormal = proxyPoint->vertices[pointIndex].Normal;
		glm::vec3 sample2TargetCamera = vCameraPos - samplePosition;

		float score = 0;
		int foundReference = 0;

		for (auto & cameraItem : obsRays[pointIndex]) {
			if (cameraItem.first.w == vCameraIndex) {
				foundReference = 1;
				continue;
			}

			glm::vec3 camera1Pos = cameraVertexVector[cameraItem.first.w].Position;
			glm::vec3 camera1Direction = cameraVertexVector[cameraItem.first.w].Normal;
			glm::vec3 sample2Camera1 = camera1Pos - samplePosition;

			float camera1Distance = glm::length(sample2Camera1);
			float camera2Distance = glm::length(sample2TargetCamera);
			float maxDistance = max(camera1Distance, camera2Distance);

			if (maxDistance > DMAX)
				continue;

			float viewAngleCos = glm::dot(glm::normalize(sample2Camera1), glm::normalize(sample2TargetCamera));
			float viewAngle = std::acos(viewAngleCos);

			// w1 = 1 / ( 1 + exp( -k1 * ( alpha - alpha1 ) ) )
			float w1 = 1 / (1 + (std::exp(-32 * (viewAngle - glm::pi<float>() / 16))));

			// w2 = min( max(d1, d2) / dmax, 1 )
			float w2 = 1 - min(maxDistance / DMAX, 1.0f);

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
	return totalScore;
};

void initRays(int vi) {
	glm::vec3 samplePos = proxyPoint->vertices[vi].Position;
	glm::vec3 sampleNormal = proxyPoint->vertices[vi].Normal;

	//Used for parallel
	vector<std::pair<glm::vec4, glm::vec4>> localObsRays;
	vector<int> localSeenSample;

	for (int j=0;j<cameraVertexVector.size();++j)
	{
		glm::vec3 direction = cameraVertexVector[j].Position - samplePos;
		
		// Not occluded
		if (strongVisible(cameraVertexVector[j].Position
			, -cameraVertexVector[j].Position, samplePos, bvhTree, DMAX))
		{
			localObsRays.push_back(std::make_pair(glm::vec4(direction,j)
				, glm::vec4(-glm::normalize(cameraVertexVector[j].Position),1)));
		}

	}

	obsRays[vi].insert(obsRays[vi].end(), localObsRays.begin(), localObsRays.end());
}

void CPathGenarateComponent::updateRays(int vCameraIndex, glm::vec3 vNewPosition) {
	for (int i = 0; i < obsRays.size(); i++)
	{
		bool visible = false;
		glm::vec3 direction = proxyPoint->vertices[i].Position - vNewPosition;
		
		visible = strongVisible(vNewPosition, -vNewPosition
			, proxyPoint->vertices[i].Position, bvhTree, DMAX);

		initRaysMutex.lock();
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
		initRaysMutex.unlock();
	}
}

glm::vec3 CPathGenarateComponent::optimizeOrientation(glm::vec3 vCameraPosition,int vCameraIndex) {
	float bestYaw, bestPitch;
	glm::vec3 bestDirection;
	float bestScore = -2;
	for (float iPitch = 0; iPitch < 90; iPitch+=20)
	{
		float directionZ =sin(glm::radians(iPitch));
		float horizenElement = cos(glm::radians(iPitch));
		for (float iYaw = 0; iYaw < 360; iYaw+=40)
		{
			glm::vec3 direction(horizenElement*cos(iYaw), horizenElement*sin(iYaw), -directionZ);
			if (glm::length(direction) - 1.0f > 1e-5)
				throw "lalala";
			float score = func(vCameraPosition, direction, vCameraIndex);
			if (score > bestScore) {
				bestScore = score;
				bestYaw = iYaw;
				bestPitch = iPitch;
				bestDirection = direction;
			}
		}
	}
	return bestDirection;
}

std::pair<glm::vec3, float> CPathGenarateComponent::downhillSimplex(std::vector<glm::vec3>* solution, size_t vCameraIndex
	, std::function<float(glm::vec3, glm::vec3, size_t)> const & func) {
	// Evaluate the 4 candidate
	float myValues[4];
	glm::vec3 cameraDirections[4];
	for (size_t i = 0; i < solution->size(); i++)
	{
		cameraDirections[i] = -solution->at(i);
		//cameraDirections[i] = optimizeOrientation(solution->at(i), vCameraIndex);
		//myValues[i] = func(solution->at(i), cameraDirections[i], vCameraIndex);
		myValues[i] = func(solution->at(i), cameraDirections[i], vCameraIndex);
	}


	std::size_t best_idx = 0;
	std::size_t lousy_idx = 0;
	std::size_t worst_idx = 0;
	for (size_t i = 1; i < 4; i++)
	{
		if (myValues[i] > myValues[best_idx])
			best_idx = i;
		if (myValues[i] < myValues[worst_idx])
			worst_idx = i;
		if (myValues[i] > myValues[worst_idx] && myValues[i] < myValues[lousy_idx])
			lousy_idx = i;
	}

	//update the camera orientation
	cameraVertexVector[vCameraIndex].Normal = cameraDirections[best_idx];

	float best_value = myValues[best_idx];
	float lousy_value = myValues[lousy_idx];
	float worst_value = myValues[worst_idx];

	glm::vec3 bestVertex = solution->at(best_idx);

	glm::vec3 medianPosition(0.0f);
	for (std::size_t k = 0; k < 4 - 1; ++k) {
		if (k != worst_idx)
			medianPosition += solution->at(k);
	}
	medianPosition /= (4 - 1);

	glm::vec3 refl = medianPosition + (medianPosition - solution->at(worst_idx));

	//float refl_value = func(refl, optimizeOrientation(refl, vCameraIndex), vCameraIndex);
	float refl_value = func(refl, -refl, vCameraIndex);

	if (refl_value > best_value) {
		glm::vec3 exp = refl + (refl - medianPosition);
		//float exp_value = func(exp, optimizeOrientation(exp, vCameraIndex), vCameraIndex);
		float exp_value = func(exp, -exp, vCameraIndex);

		if (exp_value > best_value) {
			/* Expansion */
			solution->at(worst_idx) = exp;
			return { exp, exp_value };
		}
		else {
			/* Reflection */
			solution->at(worst_idx) = refl;
			return { refl, refl_value };
		}
	}
	else {
		if (refl_value > worst_value) {
			if (refl_value > lousy_value) {
				/* Reflection */
				solution->at(worst_idx) = refl;
				return { refl, best_value };
			}
			else {
				/* Outside contraction */
				glm::vec3 con = medianPosition + (medianPosition - solution->at(worst_idx)) * 0.5f;
				//float con_value = func(con, optimizeOrientation(con,vCameraIndex), vCameraIndex);
				float con_value = func(con, -con, vCameraIndex);

				if (con_value > worst_value) {
					solution->at(worst_idx) = con;
					if (con_value > best_value) {
						return { con, con_value };
					}
					else {
						return { bestVertex, best_value };
					}
				}
				else {
					/* Shrink */
					shrink(solution, solution->at(best_idx));
					return { bestVertex, best_value };
				}
			}
		}
		else {
			/* Inside contraction */
			glm::vec3 con = medianPosition - (medianPosition - solution->at(worst_idx)) * 0.5f;
			float con_value = func(con, -con, vCameraIndex);

			if (con_value > worst_value) {
				solution->at(worst_idx) = con;
				if (con_value > best_value) {
					return { con, con_value };
				}
				else {
					return { bestVertex, best_value };
				}
			}
			else {
				/* Shrink */
				shrink(solution, solution->at(best_idx));
				return { bestVertex, best_value };
			}
		}
	}
}

void CPathGenarateComponent::optimize(size_t vCameraIndex,int vIter) {
	size_t iCameraIndex = vCameraIndex;
	
	// Optimize, if downhill failed(return 0), increase scale and reinitialize
	std::pair<glm::vec3, float> optimizeResult = downhillSimplex(&(simplexes[iCameraIndex]), iCameraIndex, func);
	if (optimizeResult.second <= 0) {
		cameraInitializeTimes[iCameraIndex] += 1;
		initializeSimplexes(SIMPLEXINITIALIZESCALE * cameraInitializeTimes[iCameraIndex]
			, cameraVertexVector[iCameraIndex].Position
			, simplexes[iCameraIndex], iCameraIndex);
		for (int m = 0; m < simplexes[iCameraIndex].size(); ++m)
		{
			cameraCandidateMesh->changeVertex(simplexes[iCameraIndex][m], 4 * iCameraIndex + m);
		}
		//Turn the original mesh orange
		cameraMesh->changeColor(glm::vec3(1.0f, 0.75f, 0.3f), iCameraIndex);
	}
	else {
		cameraInitializeTimes[iCameraIndex] = 1;
		cameraVertexVector[iCameraIndex].Position = optimizeResult.first;
		cameraAdjustMesh->changeVertex(optimizeResult.first, iCameraIndex);
		for (int m = 0; m < simplexes[iCameraIndex].size(); ++m)
		{
			cameraCandidateMesh->changeVertex(glm::vec3(0), 4 * iCameraIndex + m);
		}
		//Turn the original mesh red
		cameraMesh->changeColor(glm::vec3(1.0f, 0.0f, 0.0f), iCameraIndex);
	}
	updateRays(iCameraIndex, optimizeResult.first);

	cout << "No." << vIter << " iter:" << optimizeResult.second << endl;
}

void CPathGenarateComponent::optimize_nadir() {
	obsRays.resize(proxyPoint->vertices.size());
	for (size_t i = 0; i < obsRays.size();++i)
		obsRays[i].reserve(cameraVertexVector.size());
	
	reconstructionScore.resize(proxyPoint->vertices.size(),0);
	bvhTree = new BVHAccel(this->m_Scene->m_Models.at("proxy_mesh")->meshes);

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
		int x = floor((proxyPoint->vertices[i].Position.x - proxyPoint->bounds.pMin[0])/ xDim * heightMapWidth);
		int y = floor((proxyPoint->vertices[i].Position.y - proxyPoint->bounds.pMin[1]) /yDim*heightMapHeight);
		x = (x >= heightMapWidth ? heightMapWidth-1 : x);
		y = (y >= heightMapHeight ? heightMapWidth-1 : y);
		
		if (airspace.data[y*heightMapWidth + x] < proxyPoint->vertices[i].Position.z)
			airspace.data[y*heightMapWidth + x] = proxyPoint->vertices[i].Position.z;
	}

	tbb::task_scheduler_init init();
	//update the obs rays
	tbb::parallel_for(size_t(0),proxyPoint->vertices.size()
		, [=](const size_t vi)
	{
		initRays(vi);
	});
	// Initialize Simplexes
	simplexes.resize(cameraVertexVector.size());
	for (size_t i = 0; i < cameraVertexVector.size(); ++i)
	{
		initializeSimplexes(SIMPLEXINITIALIZESCALE, cameraVertexVector[i].Position, simplexes[i],i);
	}
	cameraInitializeTimes.resize(cameraVertexVector.size(), 1);

	// Start iter
	for (int iter = 0; iter < 100; ++iter) {
		vector<size_t> targetCameraIndices;

		// Select independent view to optimize parallel
		for (int iCameraIndex=0;iCameraIndex<cameraVertexVector.size();++iCameraIndex)
		{
			bool toClose = true;
			for (auto& iiTargetCamera:targetCameraIndices)
			{
				if (glm::length(cameraVertexVector[iiTargetCamera].Position 
					- cameraVertexVector[iCameraIndex].Position)<-1)
				{
					toClose=false;
					break;
				}
			}
			if(toClose)
				targetCameraIndices.push_back(iCameraIndex);
		}

		// Downhill simplex
		tbb::parallel_for(size_t(0), targetCameraIndices.size()
			, [=](size_t i)
		{
			optimize(targetCameraIndices[i],iter);
		});
		int photoID = 0;
		fileFp.open("C:/Users/vcc/Documents/repo/RENDERING/LiteS/camera.log", ios::out);
		char c[8];
		sprintf(c, "%05d", photoID);
		string photoName = string(c) + ".png,";
		for (size_t iCameraIndex = 0; iCameraIndex < cameraVertexVector.size(); iCameraIndex++) {
			fileFp << photoName << cameraVertexVector[iCameraIndex].Position.x
				<<","<< cameraVertexVector[iCameraIndex].Position.y
				<< "," << cameraVertexVector[iCameraIndex].Position.z
				<< "," << cameraVertexVector[iCameraIndex].Normal.x
				<< "," << cameraVertexVector[iCameraIndex].Normal.y
				<< "," << cameraVertexVector[iCameraIndex].Normal.z
				<< endl;

			photoID += 1;
			char s[8];
			sprintf(s, "%05d", photoID);
			photoName = string(s) + ".png,";
		}
		fileFp.flush();
	}
	fileFp.close();
}

void CPathGenarateComponent::simplexPoint() {
	obsRays.resize(proxyPoint->vertices.size());

	bvhTree = new BVHAccel(this->m_Scene->m_Models.at("proxy_mesh")->meshes);

	//update the obs rays
	tbb::parallel_for(size_t(0), proxyPoint->vertices.size()
		, [=](size_t i)
	{
		initRays(i);
	});

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


