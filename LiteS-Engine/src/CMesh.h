#ifndef CMESH_H
#define CMESH_H

//#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <algorithm>

#include <glm/glm.hpp>

#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/types.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include "CShader.h"

#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)
inline float gamma(int n) {
  return static_cast<float>((n * MachineEpsilon) / (1 - n * MachineEpsilon));
}

enum ModelType { Mesh, Window, PointCloud };

inline int maxDimension(glm::vec3 v) {
  return (v[0] > v[1]) ? ((v[0] > v[2]) ? 0 : 2) : ((v[1] > v[2]) ? 1 : 2);
}

inline float maxComponent(glm::vec3 v) {
  return std::max(std::max(v[0], v[1]), v[2]);
}

inline glm::vec3 abs(glm::vec3 v) {
  return glm::vec3(std::abs(v[0]), std::abs(v[1]), std::abs(v[2]));
}

inline glm::vec3 Permute(const glm::vec3& p, int x, int y, int z) {
  return glm::vec3(p[x], p[y], p[z]);
}

// Ray Declarations
struct Ray {
  // Ray Public Methods
  Ray() : tMax(INFINITY), tMin(1e-5f) {}
  Ray(const glm::vec3& o, const glm::vec3& d)
      : o(o), d(d), tMax(INFINITY), tMin(1e-5f) {
    this->d = glm::normalize(d);
  }
  glm::vec3 operator()(float t) const { return o + d * t; }

  // Ray Public Data
  glm::vec3 o;
  glm::vec3 d;
  float tMax;
  float tMin;
};

struct Bounds3f {
  glm::vec3 pMin, pMax;

  Bounds3f() : pMax(glm::vec3(0, 0, 0)), pMin(glm::vec3(0, 0, 0)) {}
  Bounds3f(const glm::vec3 p) : pMin(p), pMax(p) {}

  Bounds3f(const glm::vec3& pMin, const glm::vec3& pMax)
      : pMin(pMin), pMax(pMax) {}

  Bounds3f unionBounds(const Bounds3f& b) {
    if (pMax == glm::vec3(0, 0, 0) && pMin == glm::vec3(0, 0, 0)) return b;
    glm::vec3 x(std::min(b.pMin[0], pMin[0]), std::min(b.pMin[1], pMin[1]),
                std::min(b.pMin[2], pMin[2]));
    glm::vec3 y(std::max(b.pMax[0], pMax[0]), std::max(b.pMax[1], pMax[1]),
                std::max(b.pMax[2], pMax[2]));
    return Bounds3f(x, y);
  }

  int MaximumExtent() const {
    glm::vec3 d = pMax - pMin;
    if (d[0] > d[1] && d[0] > d[2])
      return 0;
    else if (d[1] > d[2])
      return 1;
    else
      return 2;
  }

  glm::vec3 Offset(const glm::vec3& p) const {
    glm::vec3 o = p - pMin;
    if (pMax[0] > pMin[0]) o[0] /= pMax[0] - pMin[0];
    if (pMax[1] > pMin[1]) o[1] /= pMax[1] - pMin[1];
    if (pMax[2] > pMin[2]) o[2] /= pMax[2] - pMin[2];
    return o;
  }

  bool inside(const glm::vec3& p) const {
    return (p.x < pMax.x && p.x > pMin.x && p.y < pMax.y && p.y > pMin.y &&
            p.z < pMax.z && p.z > pMin.z);
  }

  glm::vec3 getCentroid() { return (pMin + pMax) * 0.5f; }

  float SurfaceArea() const {
    glm::vec3 d = pMax - pMin;
    return 2 * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
  }

  glm::vec3 ClosestPoint(glm::vec3 const& v) {
    glm::vec3 ret;
    for (int i = 0; i < 3; ++i) {
      ret[i] = std::max(pMin[i], std::min(v[i], pMax[i]));
    }
    return ret;
  }

  bool IntersectP(const Ray& ray, const glm::vec3& invDir,
                  const int dirIsNeg[3]) const {
    // Check for ray intersection against $x$ and $y$ slabs
    float tMin = ((dirIsNeg[0] ? pMax : pMin)[0] - ray.o[0]) * invDir[0];
    float tMax = ((dirIsNeg[0] ? pMin : pMax)[0] - ray.o[0]) * invDir[0];
    float tyMin = ((dirIsNeg[1] ? pMax : pMin)[1] - ray.o[1]) * invDir[1];
    float tyMax = ((dirIsNeg[1] ? pMin : pMax)[1] - ray.o[1]) * invDir[1];

    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    float tzMin = ((dirIsNeg[2] ? pMax : pMin)[2] - ray.o[2]) * invDir[2];
    float tzMax = ((dirIsNeg[2] ? pMin : pMax)[2] - ray.o[2]) * invDir[2];

    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
  }

  bool IntersectP(const Ray& ray, float* hitt0, float* hitt1) const {
    float t0 = 0, t1 = INFINITY;
    for (int i = 0; i < 3; ++i) {
      // Update interval for _i_th bounding box slab
      float invRayDir = 1 / ray.d[i];
      float tNear = (pMin[i] - ray.o[i]) * invRayDir;
      float tFar = (pMax[i] - ray.o[i]) * invRayDir;

      // Update parametric interval from slab intersection $t$ values
      if (tNear > tFar) std::swap(tNear, tFar);

      // Update _tFar_ to ensure robust ray--bounds intersection
      tFar *= 1 + 2 * gamma(3);
      t0 = tNear > t0 ? tNear : t0;
      t1 = tFar < t1 ? tFar : t1;
      if (t0 > t1) return false;
    }
    if (hitt0) *hitt0 = t0;
    if (hitt1) *hitt1 = t1;
    return true;
  }
};

struct SurfaceInteraction {
  glm::vec3 pHit;
  float t;
  SurfaceInteraction() {
    t = std::numeric_limits<float>::max();
    pHit = glm::vec3(0.f);
  }
  SurfaceInteraction(glm::vec3 pHit, float t) : pHit(pHit), t(t) {}
};

struct Vertex {
  glm::vec3 Position;
  glm::vec3 Normal;
  glm::vec2 TexCoords;
  glm::vec3 Color;

  Vertex()
      : Position(glm::vec3(0, 0, 0)),
        Normal(glm::vec3(0, 0, 0)),
        TexCoords(glm::vec2(0, 0)),
        Color(glm::vec3(1.f, 1.f, 1.f)) {}

  Vertex(glm::vec3 vPosition, glm::vec3 vNormal, glm::vec3 vColor)
      : Position(vPosition),
        Normal(vNormal),
        TexCoords(glm::vec2(0, 0)),
        Color(vColor) {}
};

struct Texture {
  unsigned int id;
  std::string type;
  aiString path;
};

struct MeshMaterial {
  MeshMaterial() {
    diffuse = glm::vec3(0.0f);
    specular = glm::vec3(0.0f);
    shininess = 1;
    shadingModel = 0;
    opacity = 1;
    wireframe = 0;
    name = "";
  }
  glm::vec3 diffuse;
  glm::vec3 specular;
  float shininess;
  int shadingModel;
  float opacity;
  int wireframe;
  std::string name;
};

struct Tri {
  Tri() {}
  Tri(Vertex vVertex1, Vertex vVertex3, Vertex vVertex2)
      : v1(vVertex1), v2(vVertex2), v3(vVertex3) {
    glm::vec3 pmin = v1.Position, pmax = v1.Position;
    pmin[0] = std::min(pmin[0], v2.Position[0]);
    pmin[0] = std::min(pmin[0], v3.Position[0]);
    pmin[1] = std::min(pmin[1], v2.Position[1]);
    pmin[1] = std::min(pmin[1], v3.Position[1]);
    pmin[2] = std::min(pmin[2], v2.Position[2]);
    pmin[2] = std::min(pmin[2], v3.Position[2]);

    pmax[0] = std::max(pmax[0], v2.Position[0]);
    pmax[0] = std::max(pmax[0], v3.Position[0]);
    pmax[1] = std::max(pmax[1], v2.Position[1]);
    pmax[1] = std::max(pmax[1], v3.Position[1]);
    pmax[2] = std::max(pmax[2], v2.Position[2]);
    pmax[2] = std::max(pmax[2], v3.Position[2]);
    bounds = Bounds3f();
    bounds.pMin = pmin;
    bounds.pMax = pmax;
  }

  Vertex v1;
  Vertex v2;
  Vertex v3;
  Bounds3f bounds;

  glm::vec3 closetPoint(const glm::vec3& v) {
    glm::vec3 edge0 = v2.Position - v1.Position;
    glm::vec3 edge1 = v3.Position - v1.Position;
    glm::vec3 v0 = v1.Position - v;

    float a =glm::dot(edge0,edge0);
    float b = glm::dot(edge0,edge1);
    float c =glm::dot(edge1,edge1);
    float d =glm::dot(edge0,v0);
    float e =glm::dot(edge1,v0);

    float det = a * c - b * b;
    float s = b * e - c * d;
    float t = b * d - a * e;

    if (s + t < det) {
      if (s < 0.f) {
        if (t < 0.f) {
          if (d < 0.f) {
            s = std::clamp(-d / a, 0.f, 1.f);
            t = 0.f;
          } else {
            s = 0.f;
            t = std::clamp(-e / c, 0.f, 1.f);
          }
        } else {
          s = 0.f;
          t = std::clamp(-e / c, 0.f, 1.f);
        }
      } else if (t < 0.f) {
        s = std::clamp(-d / a, 0.f, 1.f);
        t = 0.f;
      } else {
        float invDet = 1.f / det;
        s *= invDet;
        t *= invDet;
      }
    } else {
      if (s < 0.f) {
        float tmp0 = b + d;
        float tmp1 = c + e;
        if (tmp1 > tmp0) {
          float numer = tmp1 - tmp0;
          float denom = a - 2 * b + c;
          s = std::clamp(numer / denom, 0.f, 1.f);
          t = 1 - s;
        } else {
          t = std::clamp(-e / c, 0.f, 1.f);
          s = 0.f;
        }
      } else if (t < 0.f) {
        if (a + d > b + e) {
          float numer = c + e - b - d;
          float denom = a - 2 * b + c;
          s = std::clamp(numer / denom, 0.f, 1.f);
          t = 1 - s;
        } else {
          s = std::clamp(-e / c, 0.f, 1.f);
          t = 0.f;
        }
      } else {
        float numer = c + e - b - d;
        float denom = a - 2 * b + c;
        s = std::clamp(numer / denom, 0.f, 1.f);
        t = 1.f - s;
      }
    }

    return v1.Position + s * edge0 + t * edge1;
  }

  bool const Intersect(Ray& ray, SurfaceInteraction* isect) const {
    // Get triangle vertices in _p1_, _p2_, and _p3_
    glm::vec3 e1 = v2.Position - v1.Position;
    glm::vec3 e2 = v3.Position - v1.Position;
    glm::vec3 s1 = glm::cross(ray.d, e2);
    float divisor = glm::dot(s1, e1);

    if (divisor == 0.) return false;
    float invDivisor = 1.f / divisor;

    // Compute first barycentric coordinate
    glm::vec3 d = ray.o - v1.Position;
    float b1 = glm::dot(d, s1) * invDivisor;
    if (b1 < 0. || b1 > 1.) return false;

    // Compute second barycentric coordinate
    glm::vec3 s2 = glm::cross(d, e1);
    float b2 = glm::dot(ray.d, s2) * invDivisor;
    if (b2 < 0. || b1 + b2 > 1.) return false;

    // Compute _t_ to intersection point
    float t = glm::dot(e2, s2) * invDivisor;
    if (t > ray.tMax || t <= ray.tMin) return false;

    if (t < isect->t) {
      isect->t = t;
      isect->pHit = ray.o + ray.d * t;
      return true;
    }

    return true;
  }
};

class CMesh {
 public:
  CMesh();
  virtual ~CMesh() = default;
  /*  Mesh Data  */
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  MeshMaterial material;
  std::vector<Texture> textures;

  /*  Render data  */
  Bounds3f bounds;
  unsigned int VAO;
  unsigned int VBO, EBO;

  glm::mat4 model;
  static GLuint boundIndex[36];
  std::vector<Vertex> NormalPoint;

  virtual void Draw(CShader* shader) = 0;

  virtual void Draw(CShader* shader, glm::mat4& vModelMatrix) = 0;

  Bounds3f getBounds() { return this->bounds; }
  glm::vec3 getCentroid() { return this->bounds.getCentroid(); }

  bool Intersect(Ray& ray, SurfaceInteraction* isect);

  bool IntersectP(const Ray& ray);

  // virtual void setMesh(std::vector<Vertex> vertices, std::vector<unsigned
  // int> indices, MeshMaterial material) = 0;

  virtual void setupMeshWithIndex() {}
  virtual void setupMesh() {}

  virtual void changeVertex(Vertex vVertex, unsigned aIndex) {}

  virtual void changePos(glm::vec3 vNewPos, unsigned aIndex) {}

  virtual void changeColor(glm::vec3 vNewColor, unsigned aIndex) {}

  virtual void changeNormal(glm::vec3 vNewNormal, unsigned aIndex) {}

  static void saveMesh(const CMesh* v_mesh, std::string v_outName);
};

#endif
