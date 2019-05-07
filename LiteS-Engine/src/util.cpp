#include "util.h"

float triangleArea(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
  glm::vec3 edge1 = v2 - v1;
  glm::vec3 edge2 = v2 - v3;

  return glm::length(glm::cross(edge1, edge2)) / 2;
}

void initializeSimplexes(float scale, glm::vec3 vPosition,
                         std::vector<glm::vec3>& solution,
                         size_t randomGenerator) {
  // Initialize the simplexes
  solution.resize(4);
  solution[0] = vPosition;
  for (size_t i = 1; i < solution.size(); i++) {
    solution[i] = solution[0] +
                  glm::linearRand(glm::vec3(-1.0f), glm::vec3(1.0f)) * scale;
  }
}

void shrink(std::vector<glm::vec3>* vSolution, glm::vec3 vPosition) {
  for (int i = 0; i < 4; ++i) {
    glm::vec3& vert = vSolution->at(i);
    vert = vPosition + (vert - vPosition) * 0.5f;
  }
}

template <size_t row, size_t col>
Eigen::Matrix<float, row, col> eigenFromGLM(glm::mat<col, row, float> vM) {
  Eigen::Matrix<float, row, col> out;
  for (size_t y = 0; y < row; y++)
    for (size_t x = 0; x < col; x++) out(y, x) = vM[x][y];
  return out;
}

template <size_t row, size_t col>
glm::mat<col, row, float> glmFromEigen(Eigen::Matrix<float, row, col> vM) {
  glm::mat<col, row, float> out;
  for (size_t y = 0; y < row; y++)
    for (size_t x = 0; x < col; x++) out[x][y] = vM(y, x);
  return out;
}

template <size_t row>
glm::vec<row, float> glmVectorFromEigen(Eigen::Matrix<float, row, 1> vM) {
  glm::vec<row, float> out;
  for (size_t y = 0; y < row; y++) out[y] = vM(y);
  return out;
}

template <size_t row>
Eigen::Matrix<float, row, 1> eigenFromGLM(glm::vec<row, float> vM) {
  Eigen::Matrix<float, row, 1> out;
  for (size_t y = 0; y < row; y++) out(y, 0) = vM[y];
  return out;
}

void postAsiaPitchYaw(float& vPitch, float& vYaw) {
  // vYaw = -vYaw;
  //vYaw = vYaw - 90.f;
  if (vPitch>10.f)
	{
    //vPitch -= 10.f;
	}
}