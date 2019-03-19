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
#include <tbb/mutex.h>
#include <iostream>

#include <glm/gtc/matrix_access.hpp>

const float DMAX = 50;
const float DMIN = 10;
const float SIMPLEXINITIALIZESCALE = 3.0f;

//
// Input Mesh
//
CMesh* proxyPoint;
CModel* proxyModel;
BVHAccel* bvhTree;

CMesh* cameraPosMesh;
vector<Vertex> cameraPosVector;
vector<Vertex> cameraAdjustVector;
CMesh* cameraCandidateMesh;
vector<float> reconstructionScore;
std::vector<std::vector<glm::vec3>> simplexes;
vector<size_t> cameraInitializeTimes;
CMesh* cameraAdjustMesh;
std::mutex eraseMutex;

const int heightMapWidth = 32;
const int heightMapHeight = 32;
My8BitRGBImage airspace;

tbb::mutex initRaysMutex;

std::ofstream fileFp;

//Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y, Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;
vector<vector<int>> seenSample;

CPathGenarateComponent::CPathGenarateComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::visualizeCamera() {
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
	Vertex tt = cameraPosVector[0];
	vector<Vertex> t;
	t.push_back(tt);
	tt.Position = glm::vec3(0, 0, 5);
	t.push_back(tt);
	cameraPosMesh = new CPointCloudMesh(t,glm::vec3(1.0f,0.0f,0.0f),30);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraPosMesh);
}

//Camera Pos, Camera Direction, Camera Index
std::function<float(glm::vec3, glm::vec3, size_t)> func =
[&](glm::vec3 const & vCameraPos, glm::vec3 vCameraDirection, size_t vCameraIndex) -> float
{
	//
	// Return -1 if to close to mesh or ground
	//
	if (vCameraPos.z < DMIN || bvhTree->KNearest(vCameraPos).first < DMIN)
		return -1;

	//
	// In airspace
	//
	int y = (vCameraPos[1] - proxyPoint->bounds.pMin[1]) / (proxyPoint->bounds.pMax[1] -proxyPoint->bounds.pMin[1]) * heightMapHeight;
	int x = (vCameraPos[0] - proxyPoint->bounds.pMin[0]) / (proxyPoint->bounds.pMax[0] - proxyPoint->bounds.pMin[0]) * heightMapWidth;
	if (vCameraPos[2] < airspace.data[heightMapWidth*y + x])
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
	//vector<int> localSeenSample;

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


	initRaysMutex.lock();
	obsRays[vi].insert(obsRays[vi].end(), localObsRays.begin(), localObsRays.end());
	initRaysMutex.unlock();
	
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

glm::vec3 optimizeOrientation(glm::vec3 vCameraPosition,int vCameraIndex) {
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

std::pair<glm::vec3, float> downhillSimplex(std::vector<glm::vec3>* solution, size_t vCameraIndex, std::function<float(glm::vec3, glm::vec3, size_t)> const & func) {
	//
	// Evaluate the 4 candidate
	//
	float myValues[4];
	glm::vec3 cameraDirections[4];
	for (size_t i = 0; i < solution->size(); i++)
	{
		//cameraDirections[i] = optimizeOrientation(solution->at(i),vCameraIndex);
		//myValues[i] = func(solution->at(i), cameraDirections[i], vCameraIndex);
		myValues[i] = func(solution->at(i), -solution->at(i), vCameraIndex);
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

void optimize(int i, vector<size_t>  m_TargetCameraIndices,int vIter) {
	size_t iCameraIndex = m_TargetCameraIndices[i];
	//
	// Optimize, if downhill failed(return 0), increase scale and reinitialize
	//
	std::pair<glm::vec3, float> optimizeResult = downhillSimplex(&(simplexes[iCameraIndex]), iCameraIndex, func);
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
	updateRays(iCameraIndex, optimizeResult.first);

	cout << "No." << vIter << " iter:" << optimizeResult.second << endl;
}

void CPathGenarateComponent::optimize_nadir() {
	cameraAdjustVector = vector<Vertex>(cameraPosVector);
	obsRays.resize(proxyPoint->vertices.size());
	for (size_t i = 0; i < obsRays.size();++i)
		obsRays[i].reserve(cameraPosVector.size());
	
	seenSample.resize(cameraPosVector.size());
	reconstructionScore.resize(proxyPoint->vertices.size(),0);
	vector<CMesh*> meshVector;
	meshVector.push_back(proxyPoint);

	proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	//proxyModel = new CModel("C:/repos/GRAPHICS/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

	//
	// Visualize
	//
	vector<Vertex> cameraCandidate;
	cameraCandidate.resize(cameraPosVector.size() * 4);
	cameraCandidateMesh = new CPointCloudMesh(cameraAdjustVector, glm::vec3(0.0f, 0.0f, 1.0f), 10);

	cameraAdjustMesh = new CPointCloudMesh(cameraAdjustVector, glm::vec3(0.0f, 1.0f, 0.0f), 20);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraAdjustMesh);
	CEngine::toAddMeshes.push_back(cameraCandidateMesh);
	lg.~lock_guard();

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

	//update the obs rays

	tbb::parallel_for(size_t(0), proxyPoint->vertices.size()
		, [=](size_t i)
	{
		initRays(i);
	});

	//
	// Initialize Simplexes
	//
	simplexes.resize(cameraPosVector.size());
	for (size_t i = 0; i < cameraPosVector.size(); ++i)
	{
		initializeSimplexes(SIMPLEXINITIALIZESCALE, cameraPosVector[i].Position, simplexes[i]);
	}
	cameraInitializeTimes.resize(cameraPosVector.size(), 1);

	//
	// Start iter
	//
	for (int iter = 0; iter < 10; ++iter) {
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

		tbb::parallel_for(size_t(0), targetCameraIndices.size()
			, [=](size_t i)
		{
			optimize(i,targetCameraIndices,iter);
		});
		int photoID = 0;
		fileFp.open("C:/Users/vcc/Documents/repo/RENDERING/LiteS/camera.log", ios::out);
		char c[8];
		sprintf(c, "%05d", photoID);
		string photoName = string(c) + ".png,";
		for (size_t iCameraIndex = 0; iCameraIndex < cameraPosVector.size(); iCameraIndex++) {
			fileFp << photoName << cameraPosVector[iCameraIndex].Position.x
				<<","<< cameraPosVector[iCameraIndex].Position.y
				<<","<< cameraPosVector[iCameraIndex].Position.z << endl;

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

	proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

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

	//optimize_nadir();

	cout << "Extra Algorithm done" << endl;

	this->waitForStepSignal();
	return;
	
}

void CPathGenarateComponent::extraInit() {}


