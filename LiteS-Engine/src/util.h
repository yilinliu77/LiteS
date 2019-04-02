#ifndef LITE_UTIL_H
#define LITE_UTIL_H
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include<glm/gtc/random.hpp>
#include<glm/ext.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include<assimp/config.h>

#include <vector>
#include <array>
#include <numeric>
#include <algorithm>

using namespace Assimp;

struct My8BitRGBImage
{
	int ncols;
	int nrows;
	float* data;

};

float triangleArea(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
	glm::vec3 edge1 = v2 - v1;
	glm::vec3 edge2 = v2 - v3;

	return glm::length(glm::cross(edge1, edge2)) / 2;
}

void saveMesh(const CMesh* v_mesh,string v_outName) {
	aiScene* scene=new aiScene;
	scene->mRootNode = new aiNode();

	scene->mMeshes = new aiMesh*[1];
	scene->mMeshes[0] = new aiMesh();
	scene->mNumMeshes = 1;

	//scene->mMaterials = new aiMaterial*[1];
	//scene->mMaterials[0] = nullptr;
	//scene->mNumMaterials = 1;

	//scene->mMaterials[0] = new aiMaterial();

	//scene->mMeshes[0]->mMaterialIndex = 0;

	scene->mRootNode->mMeshes = new unsigned int[1];
	scene->mRootNode->mMeshes[0] = 0;
	scene->mRootNode->mNumMeshes = 1;

	auto pMesh = scene->mMeshes[0];

	size_t numValidPoints = v_mesh->vertices.size();

	pMesh->mVertices = new aiVector3D[numValidPoints];
	pMesh->mNormals = new aiVector3D[numValidPoints];
	pMesh->mNumVertices = numValidPoints;

	int i = 0;
	for (auto &p : v_mesh->vertices) {
		pMesh->mVertices[i] = aiVector3D(v_mesh->vertices[i].Position.x, v_mesh->vertices[i].Position.y
			, v_mesh->vertices[i].Position.z);
		pMesh->mNormals[i] = aiVector3D(v_mesh->vertices[i].Normal.x, v_mesh->vertices[i].Normal.y
			, v_mesh->vertices[i].Normal.z);
		++i;
	}

	Assimp::Exporter *mAiExporter=new Assimp::Exporter;
	ExportProperties *properties = new ExportProperties;
	properties->SetPropertyBool(AI_CONFIG_EXPORT_POINT_CLOUDS, true);
	mAiExporter->Export(scene, "ply", v_outName, 0, properties);

	cout << mAiExporter->GetErrorString() << endl;
	//delete properties;
	return;
}

void initializeSimplexes(float scale,glm::vec3 vPosition
	, std::vector<glm::vec3>& solution,size_t randomGenerator) {
	// Initialize the simplexes
	solution.resize(4);
	solution[0] = vPosition;
	for (size_t i = 1; i < solution.size(); i++)
	{
		solution[i] = solution[0] + glm::linearRand(glm::vec3(-1.0f), glm::vec3(1.0f))*scale;
	}
}

bool strongVisible(glm::vec3 vCameraPos, glm::vec3 vCameraOrientation
	, glm::vec3 vSamplePosition, BVHAccel* vBVH, float vDMAX) {
	glm::vec3 sampleToCamera = vCameraPos - vSamplePosition;
	// Inside the frustum
	// 0.6f ~ cos(78.0f / 2 / 180.0f * pi)
	float viewAngleCos = glm::dot(-glm::normalize(sampleToCamera), glm::normalize(vCameraOrientation));
	if (viewAngleCos<0.77)
		return false;

	if (glm::length(vSamplePosition - vCameraPos) > vDMAX)
		return false;

	//Not occluded
	if (!vBVH->Visible(vCameraPos, vSamplePosition, 1.0f))
		return false;

	return true;
}

void shrink(std::vector<glm::vec3>* vSolution, glm::vec3 vPosition) {
	for (int i = 0; i < 4; ++i) {
		glm::vec3 & vert = vSolution->at(i);
		vert = vPosition + (vert - vPosition) * 0.5f;
	}
}

template<typename T>
std::vector<T> splitString(std::string str, std::string tok) {
	size_t splitPos;
	size_t psplitPos;
	istringstream ss;
	std::vector<T> out;
	T temp;
	splitPos = str.find(tok, 0);
	psplitPos = 0;
	while (splitPos!=std::string::npos)
	{
		ss.str(str.substr(psplitPos, splitPos));
		ss >> temp;
		out.push_back(temp);

		ss.str("");
		psplitPos = splitPos + 1;
		splitPos = str.find(tok, splitPos + 1);
	}
	ss.str(str.substr(psplitPos));
	ss >> temp;
	out.push_back(temp);
	return out;
}

#endif
