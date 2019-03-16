#ifndef LITE_UTIL_H
#define LITE_UTIL_H
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

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

	Assimp::Exporter mAiExporter;
	ExportProperties *properties = new ExportProperties;
	properties->SetPropertyBool(AI_CONFIG_EXPORT_POINT_CLOUDS, true);
	mAiExporter.Export(&scene, "ply", v_outName, 0, properties);

	cout << mAiExporter.GetErrorString() << endl;
	//delete properties;
	return;
}

void initializeSimplexes(float scale,glm::vec3 vPosition, std::vector<glm::vec3>& solution) {
	// 
	// Initialize the simplexes
	//
	solution.resize(4);
	glm::mat3 myIdentity(scale);
	myIdentity[2][2] = -scale;
	solution[0] = vPosition;
	for (size_t i = 1; i < solution.size(); i++)
	{
		solution[i] = solution[0] + myIdentity[i - 1];
	}
}


void shrink(std::vector<glm::vec3>* vSolution, glm::vec3 vPosition) {
	for (int i = 0; i < 4; ++i) {
		glm::vec3 & vert = vSolution->at(i);
		vert = vPosition + (vert - vPosition) * 0.5f;
	}
}

std::pair<glm::vec3, float> downhillSimplex(std::vector<glm::vec3>* solution,size_t vCameraIndex, std::function<float(glm::vec3, glm::vec3, size_t)> const & func) {
	//
	// Evaluate the 4 candidate
	//
	float myValues[4];
	for (size_t i = 0; i < solution->size(); i++)
	{
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
		if(k!=worst_idx)
			medianPosition += solution->at(k);
	}
	medianPosition /= (4 - 1);

	glm::vec3 refl = medianPosition + (medianPosition - solution->at(worst_idx));

	float refl_value = func(refl, -refl, vCameraIndex);

	if (refl_value > best_value) {
		glm::vec3 exp = refl + (refl - medianPosition);
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
	
#endif
