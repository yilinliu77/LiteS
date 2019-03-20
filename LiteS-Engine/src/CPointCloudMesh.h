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

	void processPointCloudNode(aiNode * node, const aiScene * scene);

	void setupMeshWithIndex() override;
	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader* shader, glm::mat4& vModelMatrix) override;

	void changeColor(glm::vec3 aColor, unsigned aIndex) override;
	void changeVertex(glm::vec3 vVertex, unsigned aIndex) override;

	std::mutex m_VAOMutex;
	vector<glm::vec3> pointsColorAdd;
	vector<unsigned> pointsColorIndexAdd;

	vector<glm::vec3> pointsVertexAdd;
	vector<unsigned> pointsVertexIndexAdd;

	float pointSize = -1;

};

#endif
