#ifndef CPOINTCLOUDMESH_H
#define CPOINTCLOUDMESH_H
#include "CMesh.h"
#include <mutex>

class CPointCloudMesh:public CMesh {
public:
	CPointCloudMesh(const std::vector<Vertex>& vPoints, const vector<unsigned int> &vIndices);
	CPointCloudMesh(const std::vector<Vertex>& vPoints);

	CPointCloudMesh(const std::vector<Vertex>& vPoints, const glm::vec3 vColor);

	CPointCloudMesh(const std::vector<Vertex>& vPoints, const glm::vec3 vColor, const float vPointSize);

	CPointCloudMesh(const std::string& vPath);

	vector<glm::mat4> normalMatrixes;

	void processPointCloudNode(aiNode * node, const aiScene * scene);

	void setupMeshWithIndex() override;
	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader* shader, glm::mat4& vModelMatrix) override;

	void changePos(glm::vec3 vNewPos, unsigned aIndex) override;

	void changeColor(glm::vec3 vNewColor, unsigned aIndex) override;

	void changeNormal(glm::vec3 vNewNormal, unsigned aIndex) override;

	void changeVertex(Vertex vVertex, unsigned aIndex) override;

	std::mutex m_VAOMutex;

	vector<Vertex> pointsVertexChange;
	vector<unsigned> pointsVertexChangeIndex;

	float pointSize = -1;

};

#endif
