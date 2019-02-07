#ifndef CPOINTCLOUDMESH_H
#define CPOINTCLOUDMESH_H
#include "CMesh.h"
#include <mutex>

class CPointCloudMesh:public CMesh {
public:
	CPointCloudMesh(const std::vector<Vertex>& vPoints, const vector<unsigned int> &vIndices);

	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader* shader, glm::mat4& vModelMatrix) override;

	vector<glm::vec3> pointsColor;

	void changeColor(glm::vec3 aColor, unsigned aIndex) override;

	std::mutex m_VAOMutex;
	vector<glm::vec3> pointsColorAdd;
	vector<unsigned> pointsIndexAdd;

};

#endif
