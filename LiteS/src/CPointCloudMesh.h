#ifndef CPOINTCLOUDMESH_H
#define CPOINTCLOUDMESH_H
#include "CMesh.h"

class CPointCloudMesh:public CMesh {
public:
	CPointCloudMesh(const std::vector<Vertex>& vPoints, const vector<unsigned int> &vIndices);

	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader* shader, glm::mat4& vModelMatrix);

};

#endif
