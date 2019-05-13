#ifndef CPOINTCLOUDMESH_H
#define CPOINTCLOUDMESH_H
#include <mutex>
#include "CMesh.h"

class CPointCloudMesh : public CMesh {
 public:
  CPointCloudMesh(const std::vector<Vertex>& vPoints,
                  const glm::vec3 vColor = glm::vec3(1.f, 0.f, 0.f),
                  const float vPointSize = 3);

  CPointCloudMesh(const std::string& vPath, bool vIsRender = false);

  std::vector<glm::mat4> normalMatrixes;

  void setupMesh() override;
  void Draw(CShader* shader, bool vIsNormal) override;
  void Draw(CShader* shader, glm::mat4& vModelMatrix) override;

  void changePos(glm::vec3 vNewPos, unsigned aIndex) override;

  void changeColor(glm::vec3 vNewColor, unsigned aIndex) override;

  void changeNormal(glm::vec3 vNewNormal, unsigned aIndex) override;

  void changeVertex(Vertex vVertex, unsigned aIndex) override;

  std::mutex m_VAOMutex;

  std::vector<Vertex> pointsVertexChange;
  std::vector<unsigned> pointsVertexChangeIndex;

  float pointSize = -1;
};

#endif
