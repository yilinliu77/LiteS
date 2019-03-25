#ifndef CTRIMESH_H
#define CTRIMESH_H
#include "CMesh.h"

class CTriMesh:public CMesh {
public:
	CTriMesh(const std::vector<Vertex>& vPoints, const vector<unsigned> &vIndices);
	CTriMesh();
	CTriMesh(vector<Vertex> &vertices, vector<unsigned> &indices
		, MeshMaterial &material, vector<Texture> &textures);
	CTriMesh(glm::vec3 c, float edge);
	void setMesh(vector<Vertex> vertices, vector<unsigned> indices, MeshMaterial material);
	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader * shader, glm::mat4 & vModelMatrix) override;
	void changeVertex(Vertex aVertexPosition, unsigned aIndex) override;
};
	

#endif
