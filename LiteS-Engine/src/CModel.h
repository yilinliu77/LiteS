#pragma once
#ifndef CMODEL_H
#define CMODEL_H

#include "CMesh.h"

using namespace std;

class CModel {
public:
	vector<CMesh*> meshes;
	string directory;
	bool gammaCorrection;
	bool isRender;

	/*  Functions   */
	// constructor, expects a filepath to a 3D model.
	CModel(string const &path, ModelType v_type, bool isRender);

	CModel();

	void loadModel(string const &path);

	void loadPointCloud(string const &path);
	void processPointCloudNode(aiNode* node, const aiScene* scene);

	void draw(CShader* vShader);

	void draw(CShader* vShader, glm::mat4& vModelMatrix);

	ModelType m_modelType;

	//bool usedTexture;
	vector<Texture> textures_loaded;
	// processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
	void processNode(aiNode *node, const aiScene *scene);

	CMesh* processMesh(aiMesh *mesh, const aiScene *scene);

	vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName);
};

#endif