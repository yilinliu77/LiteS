#ifndef CCPRIMITIVE_H
#define CCPRIMITIVE_H

#include "CCDataStructure.h"
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include <math_functions.h>

namespace CCDataStructure {

struct Bounds {
  glm::vec3 pMin, pMax;

  Bounds() {}

  Bounds(const glm::vec3 pMin, const glm::vec3 pMax) : pMin(pMin), pMax(pMax) {}

  Bounds unionBounds(const Bounds& b) {
    if (pMax == glm::vec3(0, 0, 0) && pMin == glm::vec3(0, 0, 0)) return b;
    glm::vec3 x(min(b.pMin[0], pMin[0]), min(b.pMin[1], pMin[1]),
                min(b.pMin[2], pMin[2]));
    glm::vec3 y(max(b.pMax[0], pMax[0]), max(b.pMax[1], pMax[1]),
                max(b.pMax[2], pMax[2]));
    return Bounds(x, y);
  }


  bool inside(const glm::vec3& p) const {
    return (p.x < pMax.x && p.x > pMin.x && p.y < pMax.y && p.y > pMin.y &&
            p.z < pMax.z && p.z > pMin.z);
  }

  glm::vec3 getCentroid();

  glm::vec3 ClosestPoint(glm::vec3 const& v) {
    glm::vec3 ret;
    for (int i = 0; i < 3; ++i) {
      ret[i] = max(pMin[i], min(v[i], pMax[i]));
    }
    return ret;
  }

  CUDA_CALLABLE_MEMBER
  bool Intersect(const Ray& ray, float* hitt0, float* hitt1) const;

};

struct Tri {
  Tri();
  Tri(Vertex vVertex1, Vertex vVertex3, Vertex vVertex2);

  Vertex v1;
  Vertex v2;
  Vertex v3;
  Bounds3f bounds;

  const glm::vec3 closetPoint(const glm::vec3& v) const;

  CUDA_CALLABLE_MEMBER
  bool Intersect(Ray& ray, SurfaceInteraction* isect) const;
};

}  // namespace CCDataStructure

#endif