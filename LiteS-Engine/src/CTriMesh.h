#ifndef CTRIMESH_H
#define CTRIMESH_H
#include "CMesh.h"

class CTriMesh:public CMesh {
public:
  CTriMesh(const std::vector<Vertex> &vPoints,
           const std::vector<unsigned> &vIndices);
 CTriMesh(const std::vector<Vertex> &vPoints);
 CTriMesh(const std::string &vPath, bool vIsRender = false);
	CTriMesh();
  CTriMesh(std::vector<Vertex> &vertices, std::vector<unsigned> &indices,
                 MeshMaterial &material, std::vector<Texture> &textures);
	void setMesh(std::vector<Vertex> vertices, std::vector<unsigned> indices, MeshMaterial material);
  void setupMeshNoIndices();
	void setupMesh() override;
        void Draw(CShader *shader, bool vIsNormal) override;
	void Draw(CShader * shader, glm::mat4 & vModelMatrix) override;
	void changeVertex(Vertex aVertexPosition, unsigned aIndex) override;
};
	

#endif
