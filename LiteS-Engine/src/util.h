#ifndef LITE_UTIL_H
#define LITE_UTIL_H
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include<assimp/config.h>

using namespace Assimp;

float triangleArea(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
	glm::vec3 edge1 = v2 - v1;
	glm::vec3 edge2 = v2 - v3;

	return glm::length(glm::cross(edge1, edge2)) / 2;
}

void saveMesh(CMesh* v_mesh,string v_outName) {
	aiScene scene;
	scene.mRootNode = new aiNode();

	scene.mMeshes = new aiMesh*[1];
	scene.mMeshes[0] = nullptr;
	scene.mNumMeshes = 1;

	scene.mMaterials = new aiMaterial*[1];
	scene.mMaterials[0] = nullptr;
	scene.mNumMaterials = 1;

	scene.mMaterials[0] = new aiMaterial();

	scene.mMeshes[0] = new aiMesh();
	scene.mMeshes[0]->mMaterialIndex = 0;

	scene.mRootNode->mMeshes = new unsigned int[1];
	scene.mRootNode->mMeshes[0] = 0;
	scene.mRootNode->mNumMeshes = 1;

	auto pMesh = scene.mMeshes[0];

	size_t numValidPoints = v_mesh->vertices.size();

	pMesh->mVertices = new aiVector3D[numValidPoints];
	pMesh->mNumVertices = numValidPoints;

	int i = 0;
	for (auto &p : v_mesh->vertices) {
		pMesh->mVertices[i] = aiVector3D(v_mesh->vertices[i].Position.x, v_mesh->vertices[i].Position.y
			, v_mesh->vertices[i].Position.z);
		++i;
	}

	Assimp::Exporter mAiExporter;
	ExportProperties *properties = new ExportProperties;
	properties->SetPropertyBool(AI_CONFIG_EXPORT_POINT_CLOUDS, true);
	mAiExporter.Export(&scene, "ply", v_outName, 0, properties);

	cout << mAiExporter.GetErrorString() << endl;
	//delete properties;
	return;
}
	
#endif
