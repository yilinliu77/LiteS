#include "CPathGenarateComponent.h"
#include "CPointCloudMesh.h"
#include <CEngine.h>
#include "CBVHACCEL.h"

#include "util.h"
#include <set>
#include <random>
#include <array>

CMesh* pointCloudMesh;
BVHAccel* bvhTree;
CMesh* cameraPosMesh;
vector<Vertex> cameraPosVector;

//Which camera can see an specific sample
vector<vector<glm::vec3>> obsRays;
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
	pointCloudMesh = DisplayPass->m_Models[0]->meshes[0];

	glm::vec3 mesh_dim = pointCloudMesh->bounds.pMax - pointCloudMesh->bounds.pMin;
	float max_height = pointCloudMesh->bounds.pMax[2] + 15;

	float step = 15;

	glm::vec3 cameraPos = pointCloudMesh->bounds.pMin;
	cameraPos.z = max_height;
	while (cameraPos.x< pointCloudMesh->bounds.pMax.x)
	{
		while (cameraPos.y < pointCloudMesh->bounds.pMax.y)
		{
			Vertex v;
			v.Position = cameraPos;
			cameraPosVector.push_back(v);
			cameraPos.y += step;
		}
		cameraPos.x += step;
		cameraPos.y = pointCloudMesh->bounds.pMin.y;
	}

	cameraPosMesh = new CPointCloudMesh(cameraPosVector,glm::vec3(1.0f,0.0f,0.0f));
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraPosMesh);
}

void updateRays(int vi) {
	glm::vec3 samplePos = pointCloudMesh->vertices[vi].Position;
	glm::vec3 sampleNormal = pointCloudMesh->vertices[vi].Normal;
	for (int j=0;j<cameraPosVector.size();++j)
	{
		Ray ray(cameraPosVector[j].Position, samplePos - cameraPosVector[j].Position);
		SurfaceInteraction si;
		if (bvhTree->Intersect(ray,&si))
		{
			if (glm::distance(ray.o + si.t*ray.d, samplePos) > 1e-5)
			{
				continue;
			}

			obsRays[vi].push_back(-si.t*ray.d);
			seenSample[j].push_back(vi);
		}
	}
}

void CPathGenarateComponent::optimize_nadir() {
	obsRays.resize(pointCloudMesh->vertices.size());
	seenSample.resize(cameraPosVector.size());
	vector<CMesh*> meshVector;
	meshVector.push_back(pointCloudMesh);
	bvhTree = new BVHAccel(meshVector);
	//update the obs rays
	for (int i = 0; i < pointCloudMesh->vertices.size();++i) {
		updateRays(i);
	}
}

void CPathGenarateComponent::extraAlgorithm() {
	generate_nadir();

	optimize_nadir();

	cout << "Extra Algorithm done" << endl;
	return;
	
}

void CPathGenarateComponent::extraInit() {}


