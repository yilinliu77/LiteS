#include "CBVHDisplay.h"
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
	float max_height = proxyPoint->bounds.pMax[2] + 15;

	float step = 15;

	glm::vec3 cameraPos = proxyPoint->bounds.pMin;
	cameraPos.z = max_height;
	while (cameraPos.x< proxyPoint->bounds.pMax.x)
	{
		while (cameraPos.y < proxyPoint->bounds.pMax.y)
		{
			Vertex v;
			v.Position = cameraPos;
			cameraPosVector.push_back(v);
			cameraPos.y += step;
		}
		cameraPos.x += step;
		cameraPos.y = proxyPoint->bounds.pMin.y;
	}

	cameraPosMesh = new CPointCloudMesh(cameraPosVector,glm::vec3(1.0f,0.0f,0.0f));
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
		//
		// Inside the frustum
		//

		
		//
		// Not ocluded
		//
		if (bvhTree->Visible(cameraPosVector[j], proxyPoint->vertices[vi]))
		{
			glm::vec3 direction = glm::normalize(cameraPosVector[j].Position - samplePos);
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

void CPathGenarateComponent::optimize_nadir() {
	obsRays.resize(proxyPoint->vertices.size());
	seenSample.resize(cameraPosVector.size());
	vector<CMesh*> meshVector;
	meshVector.push_back(proxyPoint);

	proxyModel = new CModel("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply", Mesh);
	bvhTree = new BVHAccel(proxyModel->meshes);

	//update the obs rays
	#pragma omp parallel for
	for (int i = 0; i < proxyPoint->vertices.size(); ++i) {
		updateRays(i);
	}
}

void CPathGenarateComponent::extraAlgorithm() {
	BVHAccel bvh = BVHAccel(this->DisplayPass->m_Models[0]->meshes);

	LinearBVHNode* nodes = bvh.getLinearNodes();

	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[64];
	while (true) {
		const LinearBVHNode *node = &nodes[currentNodeIndex];
		// Check ray against BVH node
		if (node->nObject > 0) {
			// Intersect ray with primitives in leaf BVH node
			for (int i = 0; i < node->nObject; ++i)
			{
				vector<Vertex> t;
				t.push_back(bvh.getOrderedTriangles()[node->objectOffset + i]->v1);
				t.push_back(bvh.getOrderedTriangles()[node->objectOffset + i]->v2);
				t.push_back(bvh.getOrderedTriangles()[node->objectOffset + i]->v3);
				int kk = 4;
				for (int k=0;k<3;++k)
				{
					t[k].Position.x += t[k].Position.x > 0 ? kk : (-kk);
					t[k].Position.y += t[k].Position.y > 0 ? kk : (-kk);
					t[k].Position.z += kk;
				}
				
				CMesh* newMesh = new CPointCloudMesh(t, glm::vec3(0, 1, 0));
				std::lock_guard<std::mutex> lg(CEngine::m_addMeshMutex);
				CEngine::toAddMeshes.push_back(newMesh);
			}
			if (toVisitOffset == 0) break;
			currentNodeIndex = nodesToVisit[--toVisitOffset];
		}
		else {
			// Put far BVH node on _nodesToVisit_ stack, advance to near
			// node
			nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
			currentNodeIndex = node->secondChildOffset;

		}
	}
	cout << "lalala" << endl;
	return;
	
}

void CPathGenarateComponent::extraInit() {}


