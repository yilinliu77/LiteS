#include "CPointCloudMesh.h"

CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints,
                                 const glm::vec3 vColor ,
                                 const float vPointSize)
    : CMesh(), pointSize(vPointSize) {
  this->vertices = vPoints;
  for (auto& v : vertices) v.Color = vColor;

  bounds = Bounds3f(this->vertices);
}

CPointCloudMesh::CPointCloudMesh(const std::string& vPath, bool vIsRender)
    : CMesh() {
  this->isRender = vIsRender;
  loadMeshFromFile(vPath, vIsRender);
  setupMesh();
  bounds = Bounds3f(this->vertices);
}

void CPointCloudMesh::setupMesh() {
  // create buffers/arrays
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);
  // load data into vertex buffers
  glBindBuffer(GL_ARRAY_BUFFER, VBO);

  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), nullptr,
               GL_STATIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex),
                  &vertices[0]);

  // set the vertex attribute pointers
  // vertex Positions
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, Position));
  // vertex normals
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, Normal));
  // vertex texcoords
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, TexCoords));
  // vertex color
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void*)offsetof(Vertex, Color));

  glBindVertexArray(0);
}

void CPointCloudMesh::Draw(CShader* shader,bool vIsNormal) {
  if (vIsNormal && !isRenderNormal || !isRender) return;
  std::lock_guard<std::mutex> lock(m_VAOMutex);
  // draw point cloud
  glBindVertexArray(VAO);

  if (!pointsVertexChangeIndex.empty()) {
    for (int i = 0; i < pointsVertexChange.size(); ++i) {
      glBindBuffer(GL_ARRAY_BUFFER, VBO);
      glBufferSubData(GL_ARRAY_BUFFER,
                      sizeof(Vertex) * pointsVertexChangeIndex[i],
                      sizeof(Vertex), &pointsVertexChange[i]);
    }
    pointsVertexChange.clear();
    pointsVertexChangeIndex.clear();
  }
  float nowPointSize;
  glGetFloatv(GL_POINT_SIZE, &nowPointSize);
  if (this->pointSize != -1) glPointSize(this->pointSize);
  shader->setBool("renderNormal", false);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices.size()));
  glBindVertexArray(0);

  glPointSize(nowPointSize);
}

void CPointCloudMesh::Draw(CShader* shader, glm::mat4& vModelMatrix) {}

void CPointCloudMesh::changePos(glm::vec3 vNewPos, unsigned aIndex) {
  Vertex t;
  t.Position = vNewPos;
  t.Normal = this->vertices[aIndex].Normal;
  t.Color = this->vertices[aIndex].Color;
  this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeColor(glm::vec3 vNewColor, unsigned aIndex) {
  Vertex t;
  t.Position = this->vertices[aIndex].Position;
  t.Normal = this->vertices[aIndex].Normal;
  t.Color = vNewColor;
  this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeNormal(glm::vec3 vNewNormal, unsigned aIndex) {
  Vertex t;
  t.Position = this->vertices[aIndex].Position;
  t.Normal = vNewNormal;
  t.Color = this->vertices[aIndex].Color;
  this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeVertex(Vertex vVertexPosition, unsigned aIndex) {
  std::lock_guard<std::mutex> lock(m_VAOMutex);
  this->vertices[aIndex] = vVertexPosition;
  pointsVertexChange.push_back(vVertexPosition);
  pointsVertexChangeIndex.push_back(aIndex);
}