#ifndef CMESH_H
#define CMESH_H

#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <vector>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "CShader.h"
#include <assimp/types.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using namespace std;

#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)
inline float gamma(int n) {
	return static_cast<float>((n * MachineEpsilon) / (1 - n * MachineEpsilon));
}

enum ModelType {
	Mesh, Window, PointCloud
};

// Ray Declarations
struct Ray {
	// Ray Public Methods
	Ray():tMax(INFINITY){}
	Ray(const glm::vec3 &o, const glm::vec3 &d)
		: o(o), d(d), tMax(INFINITY){}
	glm::vec3 operator()(float t) const { return o + d * t; }
	friend std::ostream &operator<<(std::ostream &os, const Ray &r) {
		os << "[o=" << glm::to_string(r.o)<< ", d=" << glm::to_string(r.d) << "]";
		return os;
	}

	// Ray Public Data
	glm::vec3 o;
	glm::vec3 d;
	float tMax;
};

struct Bounds3f {
	glm::vec3 pMin, pMax;

	Bounds3f() : pMax(glm::vec3(0,0,0)), pMin(glm::vec3(0, 0, 0)) {}
	Bounds3f(const glm::vec3 p) : pMin(p), pMax(p) {}

	Bounds3f(const glm::vec3 &pMin, const glm::vec3 &pMax)
		: pMin(pMin)
		, pMax(pMax) {}

	Bounds3f unionBounds(const Bounds3f &b) {
		if (pMax == glm::vec3(0,0,0) && pMin == glm::vec3(0,0,0))
			return b;
		glm::vec3 x(
			std::min(b.pMin[0], pMin[0]),
			std::min(b.pMin[1], pMin[1]),
			std::min(b.pMin[2], pMin[2]));
		glm::vec3 y(
			std::max(b.pMax[0], pMax[0]),
			std::max(b.pMax[1], pMax[1]),
			std::max(b.pMax[2], pMax[2]));
		return Bounds3f(x,y);
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

	glm::vec3 getCentroid() {
		return (pMin + pMax) * 0.5f;
	}

	float SurfaceArea() const {
		glm::vec3 d = pMax - pMin;
		return 2 * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
	}

	bool IntersectP(const Ray &ray, const glm::vec3 &invDir,
		const int dirIsNeg[3]) const {
		// Check for ray intersection against $x$ and $y$ slabs
		float tMin = ((dirIsNeg[0]? pMax : pMin)[0] - ray.o[0]) * invDir[0];
		float tMax = ((dirIsNeg[0] ? pMin : pMax)[0] - ray.o[0]) * invDir[0];
		float tyMin = ((dirIsNeg[1] ? pMin : pMax)[1] - ray.o[1]) * invDir[1];
		float tyMax = ((dirIsNeg[1] ? pMin : pMax)[1] - ray.o[1]) * invDir[1];

		// Update _tMax_ and _tyMax_ to ensure robust bounds intersection
		tMax *= 1 + 2 * gamma(3);
		tyMax *= 1 + 2 * gamma(3);
		if (tMin > tyMax || tyMin > tMax) return false;
		if (tyMin > tMin) tMin = tyMin;
		if (tyMax < tMax) tMax = tyMax;

		// Check for ray intersection against $z$ slab
		float tzMin = ((dirIsNeg[2] ? pMin : pMax)[2] - ray.o[2]) * invDir[2];
		float tzMax = ((dirIsNeg[2] ? pMin : pMax)[2] - ray.o[2]) * invDir[2];

		// Update _tzMax_ to ensure robust bounds intersection
		tzMax *= 1 + 2 * gamma(3);
		if (tMin > tzMax || tzMin > tMax) return false;
		if (tzMin > tMin) tMin = tzMin;
		if (tzMax < tMax) tMax = tzMax;
		return (tMin < ray.tMax) && (tMax > 0);
	}

	bool IntersectP(const Ray &ray, float *hitt0,
		float *hitt1) const {
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
	SurfaceInteraction() {}
	SurfaceInteraction(glm::vec3 pHit, float t):pHit(pHit),t(t){}
};

struct Vertex {
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec2 TexCoords;
};

struct Texture {
	unsigned int id;
	string type;
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
	string name;
};

inline int maxDimension(glm::vec3 v) {
	return (v[0] > v[1]) ? ((v[0] > v[2]) ? 0 : 2) : ((v[1] > v[2]) ? 1 : 2);
}

inline float maxComponent(glm::vec3 v) {
	return max(std::max(v[0],v[1]),v[2]);
}

inline glm::vec3 abs(glm::vec3 v) {
	return glm::vec3(std::abs(v[0]), std::abs(v[1]), std::abs(v[2]));
}

class CMesh {
public:
	CMesh();
	virtual ~CMesh() = default;
	/*  Mesh Data  */
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	MeshMaterial material;
	vector<Texture> textures;
	unsigned int VAO;

	/*  Render data  */
	Bounds3f bounds;
	unsigned int VBO, EBO;
	glm::mat4 model;
	static GLuint boundIndex[36];

	virtual void Draw(CShader * shader) = 0;

	virtual void Draw(CShader* shader, glm::mat4& vModelMatrix) = 0;

	Bounds3f getBounds() { return this->bounds; }
	glm::vec3 getCentroid() { return this->bounds.getCentroid(); }

	bool Intersect(Ray& ray, SurfaceInteraction* isect);

	bool IntersectP(const Ray& ray);

	//virtual void setMesh(vector<Vertex> vertices, vector<unsigned int> indices, MeshMaterial material) = 0;

	virtual void setupMeshWithIndex() {}
	virtual void setupMesh() {}

	virtual void changeColor(glm::vec3 aColor, unsigned aIndex) = 0;

};

#endif
