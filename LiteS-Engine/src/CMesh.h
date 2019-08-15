#ifndef CMESH_H
#define CMESH_H

//#define GLM_ENABLE_EXPERIMENTAL

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/types.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include "CShader.h"

#ifdef LITES_CUDA_ON

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#define LITES_DEVICE __device__ __host__
#else
#define LITES_DEVICE 
#endif


#define MachineEpsilon (1e-6 * 0.5)
#define FloatNAI 1e6f
 inline float gamma(int n);

inline float myClamp(float v, float a, float b) {
  if (v < a)
    return a;
  else if (v > b)
    return b;
  else
    return v;
}

enum ModelType { Mesh, Window, PointCloud };

inline int maxDimension(glm::vec3 v) {
  return (v[0] > v[1]) ? ((v[0] > v[2]) ? 0 : 2) : ((v[1] > v[2]) ? 1 : 2);
}

inline float maxComponent(glm::vec3 v) {
  return glm::max(glm::max(v[0], v[1]), v[2]);
}

inline glm::vec3 abs(glm::vec3 v) {
  return glm::vec3(glm::abs(v[0]), glm::abs(v[1]), glm::abs(v[2]));
}

inline glm::vec3 Permute(const glm::vec3& p, int x, int y, int z) {
  return glm::vec3(p[x], p[y], p[z]);
}

// Ray Declarations
struct Ray {
  // Ray Public Methods
  LITES_DEVICE Ray();
  LITES_DEVICE Ray(const glm::vec3& o, const glm::vec3& d);
  LITES_DEVICE glm::vec3 operator()(float t) const;

  // Ray Public Data
  glm::vec3 o;
  glm::vec3 d;
  float tMax;
  float tMin;
};

struct SurfaceInteraction {
  glm::vec3 pHit;
  float t;
  LITES_DEVICE SurfaceInteraction();
  LITES_DEVICE SurfaceInteraction(glm::vec3 pHit, float t);
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

struct Bounds3f {
  glm::vec3 pMin, pMax;

  Bounds3f() : pMax(glm::vec3(0, 0, 0)), pMin(glm::vec3(0, 0, 0)) {}
  Bounds3f(const glm::vec3 p) : pMin(p), pMax(p) {}

  Bounds3f(const glm::vec3& pMin, const glm::vec3& pMax)
      : pMin(pMin), pMax(pMax) {}

  Bounds3f(std::vector<Vertex> const& vVertex) {
    pMin = glm::vec3(INFINITY, INFINITY, INFINITY);
    pMax = glm::vec3(-INFINITY, -INFINITY, -INFINITY);
    for (int i = 0; i < vVertex.size(); ++i) {
      pMin[0] = glm::min(pMin[0], vVertex[i].Position[0]);
      pMin[1] = glm::min(pMin[1], vVertex[i].Position[1]);
      pMin[2] = glm::min(pMin[2], vVertex[i].Position[2]);
      pMax[0] = glm::max(pMax[0], vVertex[i].Position[0]);
      pMax[1] = glm::max(pMax[1], vVertex[i].Position[1]);
      pMax[2] = glm::max(pMax[2], vVertex[i].Position[2]);
    }
  }

  Bounds3f unionBounds(const Bounds3f& b) {
    if (pMax == glm::vec3(0, 0, 0) && pMin == glm::vec3(0, 0, 0)) return b;
    glm::vec3 x(glm::min(b.pMin[0], pMin[0]), glm::min(b.pMin[1], pMin[1]),
                glm::min(b.pMin[2], pMin[2]));
    glm::vec3 y(glm::max(b.pMax[0], pMax[0]), glm::max(b.pMax[1], pMax[1]),
                glm::max(b.pMax[2], pMax[2]));
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
      ret[i] = glm::max(pMin[i], glm::min(v[i], pMax[i]));
    }
    return ret;
  }

  LITES_DEVICE bool Intersect(const Ray& ray, float* hitt0,
                                     float* hitt1) const;
};

struct Tri {
  Tri() {}
  Tri(Vertex vVertex1, Vertex vVertex3, Vertex vVertex2)
      : v1(vVertex1), v2(vVertex2), v3(vVertex3) {
    glm::vec3 pmin = v1.Position, pmax = v1.Position;
    pmin[0] = glm::min(pmin[0], v2.Position[0]);
    pmin[0] = glm::min(pmin[0], v3.Position[0]);
    pmin[1] = glm::min(pmin[1], v2.Position[1]);
    pmin[1] = glm::min(pmin[1], v3.Position[1]);
    pmin[2] = glm::min(pmin[2], v2.Position[2]);
    pmin[2] = glm::min(pmin[2], v3.Position[2]);

    pmax[0] = glm::max(pmax[0], v2.Position[0]);
    pmax[0] = glm::max(pmax[0], v3.Position[0]);
    pmax[1] = glm::max(pmax[1], v2.Position[1]);
    pmax[1] = glm::max(pmax[1], v3.Position[1]);
    pmax[2] = glm::max(pmax[2], v2.Position[2]);
    pmax[2] = glm::max(pmax[2], v3.Position[2]);
    bounds = Bounds3f();
    bounds.pMin = pmin;
    bounds.pMax = pmax;
  }

  Vertex v1;
  Vertex v2;
  Vertex v3;
  Bounds3f bounds;

  const glm::vec3 closetPoint(const glm::vec3& v) const {
    glm::vec3 edge0 = v2.Position - v1.Position;
    glm::vec3 edge1 = v3.Position - v1.Position;
    glm::vec3 v0 = v1.Position - v;

    float a = glm::dot(edge0, edge0);
    float b = glm::dot(edge0, edge1);
    float c = glm::dot(edge1, edge1);
    float d = glm::dot(edge0, v0);
    float e = glm::dot(edge1, v0);

    float det = a * c - b * b;
    float s = b * e - c * d;
    float t = b * d - a * e;

    if (s + t < det) {
      if (s < 0.f) {
        if (t < 0.f) {
          if (d < 0.f) {
            s = myClamp(-d / a, 0.f, 1.f);
            t = 0.f;
          } else {
            s = 0.f;
            t = myClamp(-e / c, 0.f, 1.f);
          }
        } else {
          s = 0.f;
          t = myClamp(-e / c, 0.f, 1.f);
        }
      } else if (t < 0.f) {
        s = myClamp(-d / a, 0.f, 1.f);
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
          s = myClamp(numer / denom, 0.f, 1.f);
          t = 1 - s;
        } else {
          t = myClamp(-e / c, 0.f, 1.f);
          s = 0.f;
        }
      } else if (t < 0.f) {
        if (a + d > b + e) {
          float numer = c + e - b - d;
          float denom = a - 2 * b + c;
          s = myClamp(numer / denom, 0.f, 1.f);
          t = 1 - s;
        } else {
          s = myClamp(-e / c, 0.f, 1.f);
          t = 0.f;
        }
      } else {
        float numer = c + e - b - d;
        float denom = a - 2 * b + c;
        s = myClamp(numer / denom, 0.f, 1.f);
        t = 1.f - s;
      }
    }

    return v1.Position + s * edge0 + t * edge1;
  }

  LITES_DEVICE bool Intersect(Ray& ray, SurfaceInteraction* isect) const;
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
  std::vector<Texture> textures_loaded;
  std::string directory;

  /*  Render data  */
  Bounds3f bounds;
  unsigned int VAO;
  unsigned int VBO, EBO;

  glm::mat4 model;
  static GLuint boundIndex[36];
  std::vector<Vertex> NormalPoint;

  bool isRenderNormal;
  bool isRender;
  bool isLineRender;

  virtual void Draw(CShader* shader, bool vIsNormal)=0;

  virtual void Draw(CShader* shader, glm::mat4& vModelMatrix) = 0;

  Bounds3f getBounds() { return this->bounds; }
  glm::vec3 getCentroid() { return this->bounds.getCentroid(); }

  virtual void setupMeshWithIndex() {}
  virtual void setupMesh() {}

  virtual void changeVertex(Vertex vVertex, unsigned aIndex) {}

  virtual void changePos(glm::vec3 vNewPos, unsigned aIndex) {}

  virtual void changeColor(glm::vec3 vNewColor, unsigned aIndex) {}

  virtual void changeNormal(glm::vec3 vNewNormal, unsigned aIndex) {}

  static void saveMesh(const CMesh* v_mesh, std::string v_outName);

  void loadMaterialTextures(aiMaterial* mat, aiTextureType type,
                            std::string typeName,
                            std::vector<Texture>& textures);

  void processNode(aiNode* node, const aiScene* scene);

  void loadMeshFromFile(const std::string& vPath, bool vIsRender = false);
};

#endif
