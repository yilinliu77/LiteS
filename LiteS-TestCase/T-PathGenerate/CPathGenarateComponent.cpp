#include "CPathGenarateComponent.h"
#include "CPointCloudMesh.h"
#include <CEngine.h>
#include "CBVHACCEL.h"

#include "util.h"
#include <set>
#include <random>
#include <array>


CMesh* proxyPoint;
CModel* proxyModel;
BVHAccel* bvhTree;
CMesh* cameraPosMesh;
vector<Vertex> cameraPosVector;

vector<float> reconstructionScore;


//Which camera can see an specific sample
//( (Direction.x, Direction.y, Direction.z, cameraIndex), (Normal.x, Normal.y, Normal.z, distance) )
vector<vector<std::pair<glm::vec4, glm::vec4>>> obsRays;
vector<vector<int>> seenSample;

struct My8BitRGBImage
{
	int ncols;
	int nrows;
	float* data;

};

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

	cameraPosMesh = new CPointCloudMesh(cameraPosVector,glm::vec3(1.0f,0.0f,0.0f),5);
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraPosMesh);
}

void updateRays(int vi) {
	glm::vec3 samplePos = proxyPoint->vertices[vi].Position;
	glm::vec3 sampleNormal = proxyPoint->vertices[vi].Normal;

	//Used for OpenMP
	vector<std::pair<glm::vec4, glm::vec4>> localObsRays;
	vector<int> localSeenSample;

	for (int j=0;j<cameraPosVector.size();++j)
	{
		glm::vec3 direction = glm::normalize(cameraPosVector[j].Position - samplePos);
		//
		// Inside the frustum
		//
		// 0.087f ~ cos(85.0f / 180.0f * pi)
		if(glm::dot(-direction,glm::normalize(cameraPosVector[j].Normal))<0.7)
			continue;
		
		//
		// Not occluded
		//
		Vertex temp = proxyPoint->vertices[vi];
		float tempAdd = 1;
		temp.Position.z += tempAdd;
		if(cameraPosVector[j].Position.x>0)
			temp.Position.x += tempAdd;
		else
			temp.Position.x += -tempAdd;
		if (cameraPosVector[j].Position.y > 0)
			temp.Position.y += tempAdd;
		else
			temp.Position.y += -tempAdd;

		if (bvhTree->Visible(cameraPosVector[j], temp))
		{
			localObsRays.push_back(std::make_pair(glm::vec4(direction,j), glm::vec4(sampleNormal,-1)));
			//seenSample[j].push_back(vi);
		}

		
	}

	#pragma omp critical
	{
		obsRays[vi].insert(obsRays[vi].end(), localObsRays.begin(), localObsRays.end());
		//seenSample.insert(seenSample.end(), localSeenSample.begin(), localSeenSample.end());
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
	obsRays.resize(proxyPoint->vertices.size());
	seenSample.resize(cameraPosVector.size());
	reconstructionScore.resize(proxyPoint->vertices.size());
	vector<CMesh*> meshVector;
	meshVector.push_back(proxyPoint);

	proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

	//update the obs rays
	#pragma omp parallel for
	for (int i = 0; i < proxyPoint->vertices.size(); ++i) {
		updateRays(i);
	}
	//for (size_t i = 0; i < cameraPosVector.size(); i++)
	//{
	//	optimizeOrientation(i);
	//}

	std::function<float(glm::vec3, glm::vec3,size_t)> func =
		[&](glm::vec3 const & vCameraPos,glm::vec3 vCameraDirection,size_t vCameraIndex) -> float
	{
		for (size_t pointIndex = 0; pointIndex < proxyPoint->vertices.size(); pointIndex++)
		{
			glm::vec3 samplePosition = proxyPoint->vertices[pointIndex].Position;
			glm::vec3 sampleNormal = proxyPoint->vertices[pointIndex].Normal;

			if (glm::length(samplePosition - vCameraPos) > 15.0f)
				continue;

			float score;

			for (auto & cameraItem : obsRays[pointIndex]) {
				if (cameraItem.first.w == vCameraIndex)
					continue;

				glm::vec3 camera1Pos = glm::vec3(cameraItem.first.x
					, cameraItem.first.y
					, cameraItem.first.z);
				glm::vec3 camera1Direction = cameraPosVector[cameraItem.first.w].Normal;

				glm::vec3 sample2Camera1 = camera1Pos - samplePosition;
				glm::vec3 sample2Camera2 = camera1Pos - vCameraPos;

				float viewAngle = glm::dot(glm::normalize(sample2Camera1), glm::normalize(sample2Camera2));
				
				float w1=
			}


		}
	};

	for (int iter = 0; iter < 1000; ++iter) {
		//downhillSimplex()
	}

	
	
}

void CPathGenarateComponent::extraAlgorithm() {
	generate_nadir();

	optimize_nadir();


	cout << "Extra Algorithm done" << endl;

	cameraPosMesh->changeColor(glm::vec3(0, 0, 1), 0);
	for (int i=0;i<obsRays.size();++i)
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

	this->waitForStepSignal();
	return;
	
}

void CPathGenarateComponent::extraInit() {}


