#include "CPointCloudMesh.h"


CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints, const vector<unsigned> &vIndices) {
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

void CPointCloudMesh::setupMesh() {
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

	glBindVertexArray(0);
}

void CPointCloudMesh::Draw(CShader* shader) {

	// draw point cloud
	glBindVertexArray(VAO);
	glDrawArrays(GL_POINTS,0, vertices.size());

	glBindVertexArray(0);

}

void CPointCloudMesh::Draw(CShader* shader, glm::mat4& vModelMatrix) {
}
