#ifndef CTRIMESH_H
#define CTRIMESH_H
#include "CMesh.h"

class CTriMesh:public CMesh {
public:
  CTriMesh(const std::vector<Vertex> &vPoints,
           const std::vector<unsigned> &vIndices);
	CTriMesh();
  CTriMesh(std::vector<Vertex> &vertices, std::vector<unsigned> &indices,
                 MeshMaterial &material, std::vector<Texture> &textures);
	CTriMesh(glm::vec3 c, float edge);
	void setMesh(std::vector<Vertex> vertices, std::vector<unsigned> indices, MeshMaterial material);
	void setupMesh() override;
	void Draw(CShader* shader) override;
	void Draw(CShader * shader, glm::mat4 & vModelMatrix) override;
	void changeVertex(Vertex aVertexPosition, unsigned aIndex) override;
};
	

#endif
