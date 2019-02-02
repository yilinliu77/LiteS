#include "CMesh.h"

using namespace std;

GLuint CMesh::boundIndex[36] = {
	0, 1, 2, 0, 2, 3, 0, 5, 1, 0, 6, 5, 0, 3, 7, 0, 7, 6, 4, 5, 6, 4, 6, 7, 4, 2, 3, 4, 3, 7, 4, 5, 1, 4, 1, 2
};

CMesh::CMesh(const std::vector<Vertex>& vPoints, vector<unsigned int> vIndices) {
	this->vertices = vPoints;
	this->indices = vIndices;

	glm::vec3 pmin(INFINITY, INFINITY, INFINITY);
	glm::vec3 pmax(-INFINITY, -INFINITY, -INFINITY);
	for (int i = 0; i < vPoints.size(); ++i) {
		pmin[0] = std::min(pmin[0], vPoints[i].Position[0]);
		pmin[1] = std::min(pmin[1], vPoints[i].Position[1]);
		pmin[2] = std::min(pmin[2], vPoints[i].Position[2]);
		pmax[0] = std::max(pmax[0], vPoints[i].Position[0]);
		pmax[1] = std::max(pmax[1], vPoints[i].Position[1]);
		pmax[2] = std::max(pmax[2], vPoints[i].Position[2]);
	}
	bounds = Bounds3f();
	bounds.pMin = pmin;
	bounds.pMax = pmax;

	setupMesh();
}

CMesh::CMesh() {

	const float WindowVertices[] = {
		//2 position,2 texture
		-1, -1, 0, 0, //0
		-1, 1, 0, 1, //2
		1, -1, 1, 0, //5
		1, 1, 1, 1 //7
	};
	Vertex t;
	t.Position = glm::vec3(-1, -1, 0);
	t.TexCoords = glm::vec2(0, 0);
	this->vertices.push_back(t);

	t.Position = glm::vec3(-1, 1, 0);
	t.TexCoords = glm::vec2(0, 1);
	this->vertices.push_back(t);

	t.Position = glm::vec3(1, -1, 0);
	t.TexCoords = glm::vec2(1, 0);
	this->vertices.push_back(t);

	t.Position = glm::vec3(1, 1, 0);
	t.TexCoords = glm::vec2(1, 1);
	this->vertices.push_back(t);

	const unsigned int WindowIndex[] = { 0,2,3, 0,3,1 };

	for (int i = 0; i < 6; ++i)
		this->indices.push_back(WindowIndex[i]);
	
	setupMesh();
}

CMesh::CMesh(vector<Vertex> vertices, vector<unsigned int> indices
			 , MeshMaterial material, vector<Texture> textures) : model(glm::mat4(1.0)) {
	this->vertices = vertices;
	this->indices = indices;
	this->textures = textures;
	this->material = material;

	setupMesh();

	//deal with bounds
	float minx = 99999999.0f, miny = 99999999.0f, minz = 99999999.0f;
	float maxx = -99999999.0f, maxy = -99999999.0f, maxz = -99999999.0f;
	size_t countTemp = vertices.size();
	for (int i = 0; i < countTemp; ++i) {
		minx = vertices[i].Position.x < minx ? vertices[i].Position.x : minx;
		miny = vertices[i].Position.y < miny ? vertices[i].Position.y : miny;
		minz = vertices[i].Position.z < minz ? vertices[i].Position.z : minz;
		maxx = vertices[i].Position.x > maxx ? vertices[i].Position.x : maxx;
		maxy = vertices[i].Position.y > maxy ? vertices[i].Position.y : maxy;
		maxz = vertices[i].Position.z > maxz ? vertices[i].Position.z : maxz;
	}
	this->bounds = Bounds3f(glm::vec3(minx, miny, minz), glm::vec3(maxx, maxy, maxz));
}

CMesh::CMesh(glm::vec3 c, float edge) {
	glm::vec3 boundVerticesVector[] = {
		c + glm::vec3(edge / 2, edge / 2, edge / 2),
		glm::vec3(1, 1, 1),
		c + glm::vec3(edge / 2, -edge / 2, edge / 2),
		glm::vec3(1, -1, 1),
		c + glm::vec3(edge / 2, -edge / 2, -edge / 2),
		glm::vec3(1, -1, -1),
		c + glm::vec3(edge / 2, edge / 2, -edge / 2),
		glm::vec3(1, 1, -1),
		c + glm::vec3(-edge / 2, -edge / 2, -edge / 2),
		glm::vec3(-1, -1, -1),
		c + glm::vec3(-edge / 2, -edge / 2, edge / 2),
		glm::vec3(-1, -1, 1),
		c + glm::vec3(-edge / 2, edge / 2, edge / 2),
		glm::vec3(-1, 1, 1),
		c + glm::vec3(-edge / 2, edge / 2, -edge / 2),
		glm::vec3(-1, 1, -1)
	};
	Vertex vertex;
	for (int i = 0; i < 8; ++i) {
		vertex.Position = boundVerticesVector[2 * i];
		vertex.Normal = boundVerticesVector[2 * i + 1];
		this->vertices.push_back(vertex);
	}
	for (int i = 0; i < 36; i++)
		this->indices.push_back(boundIndex[i]);
	this->material = MeshMaterial();
	this->material.diffuse = glm::vec3(0, 1, 0);
	this->bounds = Bounds3f(c + glm::vec3(-edge / 2, -edge / 2, -edge / 2)
							, c + glm::vec3(edge / 2, edge / 2, edge / 2));
	this->model = glm::translate(glm::mat4(1), c) * glm::scale(glm::mat4(1), glm::vec3(edge, edge, edge));

	//this->UseTexture = false;
	setupMesh();
}

void CMesh::setMesh(vector<Vertex> vertices, vector<unsigned int> indices, MeshMaterial material) {
	this->vertices = vertices;
	this->indices = indices;
	this->textures = textures;
	this->material = material;

	// now that we have all the required data, set the vertex buffers and its attribute pointers.
	setupMesh();

	//deal with bounds
	float minx = 99999999.0f, miny = 99999999.0f, minz = 99999999.0f;
	float maxx = -99999999.0f, maxy = -99999999.0f, maxz = -99999999.0f;
	size_t countTemp = vertices.size();
	for (int i = 0; i < countTemp; ++i) {
		minx = vertices[i].Position.x < minx ? vertices[i].Position.x : minx;
		miny = vertices[i].Position.y < miny ? vertices[i].Position.y : miny;
		minz = vertices[i].Position.z < minz ? vertices[i].Position.z : minz;
		maxx = vertices[i].Position.x > maxx ? vertices[i].Position.x : maxx;
		maxy = vertices[i].Position.y > maxy ? vertices[i].Position.y : maxy;
		maxz = vertices[i].Position.z > maxz ? vertices[i].Position.z : maxz;
	}
	this->bounds = Bounds3f(glm::vec3(minx, miny, minz), glm::vec3(maxx, maxy, maxz));
}

void CMesh::setupMesh() {
	// create buffers/arrays
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

	glBindVertexArray(0);
}

void CMesh::Draw(CShader* shader) {

	if (this->textures.size() != 0) {
		shader->setBool("useTexture", true);

		unsigned int diffuseNr = 1;
		unsigned int specularNr = 1;
		for (unsigned int i = 0; i < textures.size(); i++) {
			glActiveTexture(GL_TEXTURE0 + i); // 在绑定之前激活相应的纹理单元
											  // 获取纹理序号（diffuse_textureN 中的 N）
			string number;
			string name = textures[i].type;
			if (name == "texture_diffuse")
				number = std::to_string(diffuseNr++);
			else if (name == "texture_specular")
				number = std::to_string(specularNr++);

			//shader->setInt((name + number).c_str(), i);
			glUniform1i(glGetUniformLocation(shader->ID, (name + number).c_str()), i);
			glBindTexture(GL_TEXTURE_2D, textures[i].id);
		}
		glActiveTexture(GL_TEXTURE0);

	}
	else {
		shader->setBool("useTexture", false);
		shader->setVec3("material.diffuse", this->material.diffuse);
		shader->setVec3("material.specular", this->material.specular);
		shader->setFloat("material.shininess", this->material.shininess);
	}

	
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

}

void CMesh::Draw(CShader* shader,glm::mat4 &vModelMatrix) {
	shader->setMat4("model", vModelMatrix);

	shader->setBool("useTexture", false);

	if (this->textures.size() != 0) {
		shader->setBool("useTexture", true);

		unsigned int diffuseNr = 1;
		unsigned int specularNr = 1;
		for (unsigned int i = 0; i < textures.size(); i++) {
			glActiveTexture(GL_TEXTURE0 + i); // 在绑定之前激活相应的纹理单元
											  // 获取纹理序号（diffuse_textureN 中的 N）
			string number;
			string name = textures[i].type;
			if (name == "texture_diffuse")
				number = std::to_string(diffuseNr++);
			else if (name == "texture_specular")
				number = std::to_string(specularNr++);

			//shader->setInt((name + number).c_str(), i);
			glUniform1i(glGetUniformLocation(shader->ID, (name + number).c_str()), i);
			glBindTexture(GL_TEXTURE_2D, textures[i].id);
		}
		glActiveTexture(GL_TEXTURE0);

	} else {
		shader->setBool("useTexture", false);
		shader->setVec3("material.diffuse", this->material.diffuse);
		shader->setVec3("material.specular", this->material.specular);
		shader->setFloat("material.shininess", this->material.shininess);
	}
		
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

bool CMesh::Intersect(Ray& ray, SurfaceInteraction* isect) const {
	for (int i = 0; i < vertices.size() / 3; ++i) {
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const glm::vec3& p0 = vertices[i * 3 + 0].Position;
		const glm::vec3& p1 = vertices[i * 3 + 1].Position;
		const glm::vec3& p2 = vertices[i * 3 + 2].Position;

		// Perform ray--triangle intersection test

		// Transform triangle vertices to ray coordinate space

		// Translate vertices based on ray origin
		glm::vec3 p0t = p0 - glm::vec3(ray.o);
		glm::vec3 p1t = p1 - glm::vec3(ray.o);
		glm::vec3 p2t = p2 - glm::vec3(ray.o);

		// Permute components of triangle vertices and ray direction
		int kz = maxDimension(abs(ray.d));
		int kx = kz + 1;
		if (kx == 3) kx = 0;
		int ky = kx + 1;
		if (ky == 3) ky = 0;
		glm::vec3 d = glm::vec3(ray.d[kx], ray.d[ky], ray.d[kz]);
		p0t = glm::vec3(p0t[kx], p0t[ky], p0t[kz]);
		p1t = glm::vec3(p1t[kx], p1t[ky], p1t[kz]);
		p2t = glm::vec3(p2t[kx], p2t[ky], p2t[kz]);

		// Apply shear transformation to translated vertex positions
		float Sx = -d[0] / d[2];
		float Sy = -d[1] / d[2];
		float Sz = 1.f / d[2];
		p0t[0] += Sx * p0t[2];
		p0t[1] += Sy * p0t[2];
		p1t[0] += Sx * p1t[2];
		p1t[1] += Sy * p1t[2];
		p2t[0] += Sx * p2t[2];
		p2t[1] += Sy * p2t[2];

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		float e0 = p1t[0] * p2t[1] - p1t[1] * p2t[0];
		float e1 = p2t[0] * p0t[1] - p2t[1] * p0t[0];
		float e2 = p0t[0] * p1t[1] - p0t[1] * p1t[0];

		// Fall back to double precision test at triangle edges
		if (sizeof(float) == sizeof(float) &&
			(e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
			double p2txp1ty = (double)p2t[0] * (double)p1t[1];
			double p2typ1tx = (double)p2t[1] * (double)p1t[0];
			e0 = (float)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t[0] * (double)p2t[1];
			double p0typ2tx = (double)p0t[1] * (double)p2t[0];
			e1 = (float)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t[0] * (double)p0t[1];
			double p1typ0tx = (double)p1t[1] * (double)p0t[0];
			e2 = (float)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		float det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t[2] *= Sz;
		p1t[2] *= Sz;
		p2t[2] *= Sz;
		float tScaled = e0 * p0t[2] + e1 * p1t[2] + e2 * p2t[2];
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		float invDet = 1 / det;
		float b0 = e0 * invDet;
		float b1 = e1 * invDet;
		float b2 = e2 * invDet;
		float t = tScaled * invDet;

		// Ensure that computed triangle $t$ is conservatively greater than zero

		// Compute $\delta_z$ term for triangle $t$ error bounds
		float maxZt = maxComponent(abs(glm::vec3(p0t[2], p1t[2], p2t[2])));
		float deltaZ = gamma(3) * maxZt;

		// Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
		float maxXt = maxComponent(abs(glm::vec3(p0t[0], p1t[0], p2t[0])));
		float maxYt = maxComponent(abs(glm::vec3(p0t[1], p1t[1], p2t[1])));
		float deltaX = gamma(5) * (maxXt + maxZt);
		float deltaY = gamma(5) * (maxYt + maxZt);

		// Compute $\delta_e$ term for triangle $t$ error bounds
		float deltaE =
			2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

		// Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
		float maxE = maxComponent(abs(glm::vec3(e0, e1, e2)));
		float deltaT = 3 *
			(gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
			std::abs(invDet);
		if (t <= deltaT) return false;

		// Interpolate $(u,v)$ parametric coordinates and hit point
		glm::vec3 pHit = b0 * p0 + b1 * p1 + b2 * p2;

		// Fill in _SurfaceInteraction_ from triangle hit
		*isect = SurfaceInteraction(pHit, t);

		ray.tMax = t;

	}

	return true;
}

bool CMesh::IntersectP(const Ray& ray) const {
	// Get triangle vertices in _p0_, _p1_, and _p2_
	for (int i = 0; i < vertices.size() / 3; ++i) {
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const glm::vec3& p0 = vertices[i * 3 + 0].Position;
		const glm::vec3& p1 = vertices[i * 3 + 1].Position;
		const glm::vec3& p2 = vertices[i * 3 + 2].Position;

		// Perform ray--triangle intersection test

		// Transform triangle vertices to ray coordinate space

		// Translate vertices based on ray origin
		glm::vec3 p0t = p0 - glm::vec3(ray.o);
		glm::vec3 p1t = p1 - glm::vec3(ray.o);
		glm::vec3 p2t = p2 - glm::vec3(ray.o);

		// Permute components of triangle vertices and ray direction
		int kz = maxDimension(abs(ray.d));
		int kx = kz + 1;
		if (kx == 3) kx = 0;
		int ky = kx + 1;
		if (ky == 3) ky = 0;
		glm::vec3 d = glm::vec3(ray.d[kx], ray.d[ky], ray.d[kz]);
		p0t = glm::vec3(p0t[kx], p0t[ky], p0t[kz]);
		p1t = glm::vec3(p1t[kx], p1t[ky], p1t[kz]);
		p2t = glm::vec3(p2t[kx], p2t[ky], p2t[kz]);

		// Apply shear transformation to translated vertex positions
		float Sx = -d[0] / d[2];
		float Sy = -d[1] / d[2];
		float Sz = 1.f / d[2];
		p0t[0] += Sx * p0t[2];
		p0t[1] += Sy * p0t[2];
		p1t[0] += Sx * p1t[2];
		p1t[1] += Sy * p1t[2];
		p2t[0] += Sx * p2t[2];
		p2t[1] += Sy * p2t[2];

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		float e0 = p1t[0] * p2t[1] - p1t[1] * p2t[0];
		float e1 = p2t[0] * p0t[1] - p2t[1] * p0t[0];
		float e2 = p0t[0] * p1t[1] - p0t[1] * p1t[0];

		// Fall back to double precision test at triangle edges
		if (sizeof(float) == sizeof(float) &&
			(e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
			double p2txp1ty = (double)p2t[0] * (double)p1t[1];
			double p2typ1tx = (double)p2t[1] * (double)p1t[0];
			e0 = (float)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t[0] * (double)p2t[1];
			double p0typ2tx = (double)p0t[1] * (double)p2t[0];
			e1 = (float)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t[0] * (double)p0t[1];
			double p1typ0tx = (double)p1t[1] * (double)p0t[0];
			e2 = (float)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		float det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t[2] *= Sz;
		p1t[2] *= Sz;
		p2t[2] *= Sz;
		float tScaled = e0 * p0t[2] + e1 * p1t[2] + e2 * p2t[2];
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		float invDet = 1 / det;
		float b0 = e0 * invDet;
		float b1 = e1 * invDet;
		float b2 = e2 * invDet;
		float t = tScaled * invDet;

		// Ensure that computed triangle $t$ is conservatively greater than zero

		// Compute $\delta_z$ term for triangle $t$ error bounds
		float maxZt = maxComponent(abs(glm::vec3(p0t[2], p1t[2], p2t[2])));
		float deltaZ = gamma(3) * maxZt;

		// Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
		float maxXt = maxComponent(abs(glm::vec3(p0t[0], p1t[0], p2t[0])));
		float maxYt = maxComponent(abs(glm::vec3(p0t[1], p1t[1], p2t[1])));
		float deltaX = gamma(5) * (maxXt + maxZt);
		float deltaY = gamma(5) * (maxYt + maxZt);

		// Compute $\delta_e$ term for triangle $t$ error bounds
		float deltaE =
			2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

		// Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
		float maxE = maxComponent(abs(glm::vec3(e0, e1, e2)));
		float deltaT = 3 *
			(gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
			std::abs(invDet);
		if (t <= deltaT) return false;

		return true;
	}
}
