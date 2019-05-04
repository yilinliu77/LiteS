#ifndef LITE_UTIL_H
#define LITE_UTIL_H
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
namespace LiteUtil {
template <typename T>
std::vector<T> splitString(std::string str, std::string tok) {
  size_t splitPos;
  size_t psplitPos;
  istringstream ss;
  std::vector<T> out;
  T temp;
  splitPos = str.find(tok, 0);
  psplitPos = 0;
  while (splitPos != std::string::npos) {
    ss.str(str.substr(psplitPos, splitPos));
    ss >> temp;
    out.push_back(temp);

    ss.str("");
    ss.clear();
    psplitPos = splitPos + 1;
    splitPos = str.find(tok, splitPos + 1);
  }
  ss.str(str.substr(psplitPos));
  ss >> temp;
  out.push_back(temp);
  return out;
}

template <typename T>
std::string numberToString(T vNumber) {
  std::stringstream ss;
  ss << vNumber;
  return ss.str();
}
}  // namespace LiteUtil
struct My8BitRGBImage {
  int ncols;
  int nrows;
  float* data;
};

float triangleArea(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);

void initializeSimplexes(float scale, glm::vec3 vPosition,
                         std::vector<glm::vec3>& solution,
                         size_t randomGenerator);
void shrink(std::vector<glm::vec3>* vSolution, glm::vec3 vPosition);

template <size_t row, size_t col>
Eigen::Matrix<float, row, col> eigenFromGLM(glm::mat<col, row, float> vM);

template <size_t row, size_t col>
glm::mat<col, row, float> glmFromEigen(Eigen::Matrix<float, row, col> vM);

template <size_t row>
glm::vec<row, float> glmVectorFromEigen(Eigen::Matrix<float, row, 1> vM);

template <size_t row>
Eigen::Matrix<float, row, 1> eigenFromGLM(glm::vec<row, float> vM);

void postAsiaPitchYaw(float& vPitch, float& vYaw);

#endif
