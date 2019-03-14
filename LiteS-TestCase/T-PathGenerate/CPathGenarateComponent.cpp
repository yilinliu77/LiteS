#include "CPathGenarateComponent.h"
#include "CPointCloudMesh.h"
#include <CEngine.h>

#include "util.h"
#include <set>
#include <random>



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
	CMesh* mesh = DisplayPass->m_Models[0]->meshes[0];

	glm::vec3 mesh_dim = mesh->bounds.pMax - mesh->bounds.pMin;
	float max_height = mesh->bounds.pMax[2] + 15;

	float step = 15;

	vector<Vertex> cameraPosVector;
	glm::vec3 cameraPos = mesh->bounds.pMin;
	cameraPos.z = max_height;
	while (cameraPos.x< mesh->bounds.pMax.x)
	{
		while (cameraPos.y < mesh->bounds.pMax.y)
		{
			Vertex v;
			v.Position = cameraPos;
			cameraPosVector.push_back(v);
			cameraPos.y += step;
		}
		cameraPos.x += step;
		cameraPos.y = mesh->bounds.pMin.y;
	}

	CMesh* cameraPoint = new CPointCloudMesh(cameraPosVector,glm::vec3(1.0f,0.0f,0.0f));
	//Lock the target arrays
	std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
	CEngine::toAddMeshes.push_back(cameraPoint);
}

void CPathGenarateComponent::optimize_nadir() {

}

void CPathGenarateComponent::extraAlgorithm() {
	generate_nadir();

	optimize_nadir();

	cout << "Extra Algorithm done" << endl;
	return;
	
}

void CPathGenarateComponent::extraInit() {}


