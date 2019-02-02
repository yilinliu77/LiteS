#pragma once

#ifndef CMESH_H
#define CMESH_H

#include <vector>
#include "CShader.h"
#include <algorithm>
#include <assimp/types.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
using namespace std;

struct Bounds3f {
	glm::vec3 pMin, pMax;

	Bounds3f() : pMax(glm::vec3(0)), pMin(glm::vec3(0)) {}
	Bounds3f(const glm::vec3 p) : pMin(p), pMax(p) {}

	Bounds3f(const glm::vec3 p1, const glm::vec3 p2)
		: pMin(glm::vec3(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)))
		, pMax(glm::vec3(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z))) {}

	Bounds3f unionBounds(const Bounds3f b) {
		if (pMax == glm::vec3(0) && pMin == glm::vec3(0))
			return b;
		return Bounds3f(
			glm::vec3(
				std::min(b.pMin.x, pMin.x),
				std::min(b.pMin.y, pMin.y),
				std::min(b.pMin.z, pMin.z))
			, glm::vec3(
				std::max(b.pMax.x, pMax.x),
				std::max(b.pMax.y, pMax.y),
				std::max(b.pMax.z, pMax.z)));
	}

	int MaximumExtent() const {
		glm::vec3 d = pMax - pMin;
		if (d.x > d.y && d.x > d.z)
			return 0;
		else if (d.y > d.z)
			return 1;
		else
			return 2;
	}

	glm::vec3 Offset(const glm::vec3& p) const {
		glm::vec3 o = p - pMin;
		if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
		if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
		if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;
		return o;
	}

	glm::vec3 getCentroid() {
		return (pMin + pMax) * 0.5f;
	}

	float SurfaceArea() const {
		glm::vec3 d = pMax - pMin;
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	//WARNING:just square
	float* getVertexData() {
		float edge = pMax[0] - pMin[0];
	}
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
		diffuse = glm::vec3(1.0f);
		specular = glm::vec3(1.0f);
		shininess = 1;
		shadingModel = 0;
		opacity = 1;
		wireframe = 0;
		name = "";
	}
	glm::vec3 diffuse ;
	glm::vec3 specular ;
	float shininess ;
	int shadingModel ;
	float opacity ;
	int wireframe ;
	string name ;
};

class CMesh {
public:
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	MeshMaterial material;
	vector<Texture> textures;
	//bool UseTexture;
	unsigned int VAO;

	CMesh(string vType);

	CMesh();

	CMesh(vector<Vertex> vertices, vector<unsigned int> indices
		  , MeshMaterial material, vector<Texture> textures);

	CMesh(glm::vec3 c, float edge);

	void Draw(CShader * shader);

	void Draw(CShader* shader, glm::mat4& vModelMatrix);

	Bounds3f getBounds();
	//glm::mat4 getModel() { return this->model; }
	glm::vec3 getCentroid();

	void setMesh(vector<Vertex> vertices, vector<unsigned int> indices, MeshMaterial material);

private:

	unsigned int VBO, EBO;
	Bounds3f bounds;
	glm::mat4 model;
	static GLuint boundIndex[36];

	void setupMesh();
};

#endif
